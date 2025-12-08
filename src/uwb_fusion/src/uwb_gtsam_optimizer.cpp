#include "uwb_fusion/uwb_gtsam_optimizer.h"

#include <tf_conversions/tf_eigen.h>
#include <cmath>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// Symbol shorthand
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace uwb_fusion {

UwbGtsamOptimizer::UwbGtsamOptimizer(ros::NodeHandle& nh,
                                     ros::NodeHandle& nh_private)
    : nh_(nh), current_key_(0), last_time_(-1.0), is_initialized_(false) {
  
  // 1. Load Parameters (Scientific Defaults)
  nh_private.param("fov_limit_deg", fov_limit_deg_, 70.0);
  nh_private.param("noise_range_sigma", noise_range_sigma_, 0.10); // 10cm
  nh_private.param("noise_angle_sigma", noise_angle_sigma_, 5.0);  // 5 degrees
  nh_private.param("blind_cov_thresh", blind_cov_thresh_, 1.0);
  nh_private.param("map_frame", map_frame_, std::string("odom"));

  // Keyframe selection parameters
  nh_private.param("keyframe_distance_thresh", keyframe_distance_thresh_, 0.1);  // 20cm
  nh_private.param("keyframe_time_thresh", keyframe_time_thresh_, 0.1);  // 100ms

  // Z-axis constraint parameters
  nh_private.param("z_velocity_sigma", z_velocity_sigma_, 0.03);  // Weak constraint on vertical velocity (m/s)
  nh_private.param("z_acceleration_sigma", z_acceleration_sigma_, 0.05);  // Discourage sudden z changes
  
  // 2. Initialize ROS Communication
  sub_uwb_left_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/uwb/anchor_left/pose", 10,
      boost::bind(&UwbGtsamOptimizer::UwbCallback, this, _1, "anchor_left_link"));

  sub_uwb_right_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/uwb/anchor_right/pose", 10,
      boost::bind(&UwbGtsamOptimizer::UwbCallback, this, _1, "anchor_right_link"));
  
  // For testing constant velocity model directly
  sub_simple_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/uwb/pose", 10, &UwbGtsamOptimizer::SimplePathCallback, this);

  pub_raw_path_ = nh_.advertise<nav_msgs::Path>("/uwb_raw/path", 10);
  pub_raw_attitude_ = nh_.advertise<geometry_msgs::QuaternionStamped>("/uwb_raw/attitude", 10);

  raw_path_.header.frame_id = map_frame_;  // Set the frame for the raw path

  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/uwb_optimized/odom", 10);

  pub_path_ = nh_.advertise<nav_msgs::Path>("/uwb_optimized/path", 10);
  pub_attitude_ = nh_.advertise<geometry_msgs::QuaternionStamped>("/uwb_optimized/attitude", 10);

  optimized_path_.header.frame_id = map_frame_;

  // 3. Configure ISAM2
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(params);

  last_velocity_ = gtsam::Vector3::Zero();

  ROS_INFO("Stereo UWB Optimizer Initialized in frame: %s", map_frame_.c_str());
}

gtsam::SharedNoiseModel UwbGtsamOptimizer::GetDynamicNoiseModel(
    const gtsam::Point3& local_point,
    const tf::StampedTransform& tf_map_anchor) {
  
  // 1. Calculate Geometry
  double range = local_point.norm();
  double azimuth = std::atan2(local_point.y(), local_point.x());
  double angle_deg = std::abs(azimuth) * 180.0 / M_PI;

  // 2. Physics-based Hard Cutoff
  if (angle_deg > fov_limit_deg_) {
    return nullptr;
  }

  // 3. Dynamic Variance Inflation (Scientific Logic)
  // Penalize edges heavily to force reliance on stereo intersection
  double ratio = angle_deg / fov_limit_deg_;
  double edge_penalty = 1.0 + 20.0 * std::pow(ratio, 6);

  // 4. Component Variances
  // Range is precise and independent
  double var_range = std::pow(noise_range_sigma_, 2);
  
  // Lateral error = Arc Length = Range * sin(Angle_Error_Rad)
  // This correctly models that angular position is worse at distance
  double sigma_angle_rad = (noise_angle_sigma_ * M_PI / 180.0) * edge_penalty;
  double var_lateral = std::pow(range * std::sin(sigma_angle_rad), 2);
  
  // Elevation is uncertain in 2D PDoA
  double var_elevation = std::pow(1.0, 2);

  // 5. Build Local Covariance (Aligned with Anchor Boresight)
  // X = Range, Y = Lateral
  gtsam::Matrix3 local_cov = gtsam::Matrix3::Zero();
  local_cov(0, 0) = var_range;
  local_cov(1, 1) = var_lateral;
  local_cov(2, 2) = var_elevation;

  // 6. Rotate to Tracking Frame (odom/map)
  // This accounts for the robot moving and the anchor mounting angle
  tf::Matrix3x3 tf_rot = tf_map_anchor.getBasis();
  Eigen::Matrix3d rot_eigen;
  tf::matrixTFToEigen(tf_rot, rot_eigen);

  gtsam::Matrix3 global_cov = rot_eigen * local_cov * rot_eigen.transpose();

  return gtsam::noiseModel::Gaussian::Covariance(global_cov);
}

void UwbGtsamOptimizer::UwbCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg,
    const std::string& anchor_frame_id) {
  
  // 1. TF Lookup (Where was the anchor when it saw the target?)
  tf::StampedTransform tf_map_anchor;
  try {
    tf_listener_.waitForTransform(map_frame_, anchor_frame_id,
                                  msg->header.stamp, ros::Duration(0.05));
    tf_listener_.lookupTransform(map_frame_, anchor_frame_id, msg->header.stamp,
                                 tf_map_anchor);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(2, "TF Error: %s", ex.what());
    return;
  }

  double current_time = msg->header.stamp.toSec();
  gtsam::Point3 local_meas(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // 2. Get Scientific Noise Model
  auto noise_model = GetDynamicNoiseModel(local_meas, tf_map_anchor);
  if (!noise_model) return; // Blind zone

  // 3. Transform Measurement to Global Frame
  tf::Vector3 local_vec(local_meas.x(), local_meas.y(), local_meas.z());
  tf::Vector3 global_vec = tf_map_anchor * local_vec;
  gtsam::Point3 global_meas(global_vec.x(), global_vec.y(), global_vec.z());

  // 4. Update
  if (!is_initialized_) {
    InitializeSystem(current_time, global_meas);
  } else {
    double dt = current_time - last_time_;
    if (dt < 0.005) return; // Filter duplicates
    PerformUpdate(dt, global_meas, noise_model);
  }
  
  last_time_ = current_time; 
}

void UwbGtsamOptimizer::SimplePathCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  double current_time = msg->header.stamp.toSec();
  gtsam::Point3 global_meas(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Publish raw path
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.pose = msg->pose;

  raw_path_.poses.push_back(pose_stamped);
  raw_path_.header.stamp = msg->header.stamp;
  pub_raw_path_.publish(raw_path_);

  // Publish raw attitude
  geometry_msgs::QuaternionStamped attitude_msg;
  attitude_msg.header = msg->header;
  attitude_msg.quaternion = msg->pose.orientation;
  pub_raw_attitude_.publish(attitude_msg);

  // Existing logic for testing
  gtsam::Matrix3 cov = gtsam::Matrix3::Identity() * std::pow(noise_range_sigma_, 2);
  auto noise_model = gtsam::noiseModel::Gaussian::Covariance(cov);

  if (!is_initialized_) {
    InitializeSystem(current_time, global_meas);
  } else {
    double dt = current_time - last_time_;
    if (dt < 0.005) return;
    PerformUpdate(dt, global_meas, noise_model);
  }
  last_time_ = current_time;
}

void UwbGtsamOptimizer::InitializeSystem(double time, const gtsam::Point3& initial_pos) {
  // Use gtsam::Rot3() for identity to support all GTSAM versions
  last_pose_ = gtsam::Pose3(gtsam::Rot3(), initial_pos);
  
  initial_estimate_.insert(X(0), last_pose_);
  // Explicit cast to avoid Eigen template deduction errors
  initial_estimate_.insert(V(0), gtsam::Vector3(gtsam::Vector3::Zero()));

  // Prior Factors
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), last_pose_, gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(0), gtsam::Vector3::Zero(),
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));

  is_initialized_ = true;
  ROS_INFO("System Initialized.");
}

void UwbGtsamOptimizer::PerformUpdate(double dt, const gtsam::Point3& meas_global,
                                      const gtsam::SharedNoiseModel& noise_model) {
  // Check if this should be a keyframe
  double time_from_last = (last_time_ + dt) - last_keyframe_time_;
  
  if (time_from_last < keyframe_time_thresh_) {
    return;
  }
  
  // Update keyframe tracking
  last_keyframe_pos_ = meas_global;
  last_keyframe_time_ = last_time_ + dt;
  
  current_key_++;

  // A. Constant Velocity Motion Model with Z-constraint
  double pos_noise = 1.0 * dt;   // XY position uncertainty
  double vel_noise = 1.0 * dt;   // XY velocity uncertainty
  
  // Strong constraint: Z velocity should stay near previous value (smooth stairs)
  // Weak constraint: XY velocity can change freely (turning, acceleration)
  gtsam::Vector3 vel_sigmas(vel_noise, vel_noise, z_velocity_sigma_ * dt);
  graph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
      V(current_key_ - 1), V(current_key_), gtsam::Vector3::Zero(),
      gtsam::noiseModel::Diagonal::Sigmas(vel_sigmas)));

  // X(k) ~ X(k-1) + V(k-1) * dt
  gtsam::Vector3 pred_move = last_velocity_ * dt;
  gtsam::Pose3 odom_delta = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(pred_move));

  // Anisotropic process noise: Allow XY motion, discourage Z jumps
  gtsam::Vector6 pose_sigmas;
  pose_sigmas << 500.0, 500.0, 500.0,          // Rotation (completely ignored)
                 pos_noise, pos_noise,          // XY position (normal uncertainty)
                 z_acceleration_sigma_;         // Z position (very tight)
  
  auto pred_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);

  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(current_key_ - 1), X(current_key_), odom_delta, pred_noise));

  // B. Measurement Update
  gtsam::Matrix6 full_cov = gtsam::Matrix6::Zero();
  full_cov.block<3, 3>(0, 0) = gtsam::Matrix3::Identity() * 500.0;  // Ignore rotation
  
  auto gaussian_noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise_model);
  if (gaussian_noise) {
    full_cov.block<3, 3>(3, 3) = gaussian_noise->covariance();
  } else {
    full_cov.block<3, 3>(3, 3) = gtsam::Matrix3::Identity() * 0.5;
  }

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(current_key_), gtsam::Pose3(gtsam::Rot3(), meas_global),
      gtsam::noiseModel::Gaussian::Covariance(full_cov)));

  // C. REMOVE the redundant Z-velocity prior - it conflicts with the BetweenFactor
  // The BetweenFactor on velocity already handles this!

  // D. Solve
  initial_estimate_.insert(X(current_key_), last_pose_.compose(odom_delta));
  initial_estimate_.insert(V(current_key_), last_velocity_);

  isam_.update(graph_, initial_estimate_);
  isam_.update();
  
  graph_.resize(0);
  initial_estimate_.clear();

  // E. Extract Result
  gtsam::Values result = isam_.calculateEstimate();
  last_pose_ = result.at<gtsam::Pose3>(X(current_key_));
  last_velocity_ = result.at<gtsam::Vector3>(V(current_key_));

  // Hard clamp Z velocity (e.g., max 0.3 m/s for stairs)
  if (std::abs(last_velocity_(2)) > 0.1) {
    last_velocity_(2) = std::copysign(0.3, last_velocity_(2));
  }

  CheckAndPublish(last_time_ + dt);
}

void UwbGtsamOptimizer::CheckAndPublish(double time_stamp) {
  try {
    gtsam::Matrix cov = isam_.marginalCovariance(X(current_key_));
    double trace = cov(3, 3) + cov(4, 4) + cov(5, 5);

    if (trace > blind_cov_thresh_) {
      ROS_WARN_THROTTLE(2, "Blind Zone! Uncertainty: %.2f", trace);
      return;
    }

    // Publish the optimized path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(time_stamp);
    pose_stamped.header.frame_id = map_frame_;
    pose_stamped.pose.position.x = last_pose_.x();
    pose_stamped.pose.position.y = last_pose_.y();
    pose_stamped.pose.position.z = last_pose_.z();

    gtsam::Quaternion q = last_pose_.rotation().toQuaternion();
    pose_stamped.pose.orientation.w = q.w();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();

    optimized_path_.poses.push_back(pose_stamped);
    optimized_path_.header.stamp = ros::Time(time_stamp);
    pub_path_.publish(optimized_path_);

    // Publish the latest attitude
    geometry_msgs::QuaternionStamped attitude_msg;
    attitude_msg.header.stamp = ros::Time(time_stamp);
    attitude_msg.header.frame_id = map_frame_;
    attitude_msg.quaternion.w = q.w();
    attitude_msg.quaternion.x = q.x();
    attitude_msg.quaternion.y = q.y();
    attitude_msg.quaternion.z = q.z();

    pub_attitude_.publish(attitude_msg);

  } catch (gtsam::IndeterminantLinearSystemException& e) {
    ROS_ERROR("GTSAM Error: %s", e.what());
  }
}

} // namespace uwb_fusion