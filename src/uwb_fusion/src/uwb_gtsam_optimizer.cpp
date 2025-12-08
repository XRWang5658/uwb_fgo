#include "uwb_fusion/uwb_gtsam_optimizer.h"

#include <tf_conversions/tf_eigen.h>
#include <cmath>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>

// Symbol shorthand
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace uwb_fusion {

// Helper function
gtsam::Point3 positionFromPose(const gtsam::Pose3& p, gtsam::OptionalJacobian<3, 6> H) {
  return p.translation(H);
}

UwbGtsamOptimizer::UwbGtsamOptimizer(ros::NodeHandle& nh,
                                     ros::NodeHandle& nh_private)
    : nh_(nh), current_key_(0), last_time_(-1.0), is_initialized_(false) {
  
  // 1. Load Parameters
  nh_private.param("fov_limit_deg", fov_limit_deg_, 120.0);
  nh_private.param("noise_range_sigma", noise_range_sigma_, 0.10); 
  nh_private.param("noise_angle_sigma", noise_angle_sigma_, 5.0);  
  nh_private.param("blind_cov_thresh", blind_cov_thresh_, 1.0);
  nh_private.param("map_frame", map_frame_, std::string("map"));

  // Person Tracking Parameters - REASONABLE VALUES
  nh_private.param("noise_elev_sigma", noise_elev_sigma_, 30.0);    // degrees
  nh_private.param("person_vel_sigma", person_vel_sigma_, 0.5);     // m/s² acceleration
  nh_private.param("person_z_vel_sigma", person_z_vel_sigma_, 0.05); 
  
  // Low Pass Filter Gain
  nh_private.param("lpf_gain", lpf_gain_, 0.2);

  // [NEW] Synchronization parameters
  nh_private.param("measurement_sync_window", measurement_sync_window_, 0.1); // 100ms
  nh_private.param("update_rate", update_rate_, 20.0); // 20 Hz

  // Keyframe parameters
  nh_private.param("keyframe_distance_thresh", keyframe_distance_thresh_, 0.05);
  nh_private.param("keyframe_time_thresh", keyframe_time_thresh_, 0.05); 

  // Legacy parameters
  nh_private.param("z_velocity_sigma", z_velocity_sigma_, 0.03);  
  nh_private.param("z_acceleration_sigma", z_acceleration_sigma_, 0.05);  

  // Load Topic Names
  std::string topic_uwb_left, topic_uwb_right;
  nh_private.param("topic_uwb_left", topic_uwb_left, std::string("/uwb/anchor_left/raw"));
  nh_private.param("topic_uwb_right", topic_uwb_right, std::string("/uwb/anchor_right/raw"));
  
  // 2. Initialize ROS Communication
  
  // Legacy subscribers (kept for compatibility)
  sub_uwb_left_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/uwb/anchor_left/pose", 10,
      boost::bind(&UwbGtsamOptimizer::UwbCallback, this, _1, "anchor_left_link"));

  sub_uwb_right_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/uwb/anchor_right/pose", 10,
      boost::bind(&UwbGtsamOptimizer::UwbCallback, this, _1, "anchor_right_link"));
  
  sub_simple_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/uwb/pose", 10, &UwbGtsamOptimizer::SimplePathCallback, this);

  // Raw data subscribers
  ROS_INFO("Subscribing to Raw UWB Left: %s", topic_uwb_left.c_str());
  sub_raw_left_ = nh_.subscribe<std_msgs::Int32MultiArray>(
      topic_uwb_left, 10,
      boost::bind(&UwbGtsamOptimizer::RawUwbCallback, this, _1, "anchor_left_link"));

  ROS_INFO("Subscribing to Raw UWB Right: %s", topic_uwb_right.c_str());
  sub_raw_right_ = nh_.subscribe<std_msgs::Int32MultiArray>(
      topic_uwb_right, 10,
      boost::bind(&UwbGtsamOptimizer::RawUwbCallback, this, _1, "anchor_right_link"));

  // Publishers
  pub_raw_path_left_ = nh_.advertise<nav_msgs::Path>("/uwb_raw/left", 10);
  pub_raw_path_right_ = nh_.advertise<nav_msgs::Path>("/uwb_raw/right", 10);
  pub_raw_attitude_ = nh_.advertise<geometry_msgs::QuaternionStamped>("/uwb_raw/attitude", 10);
  
  raw_path_left_.header.frame_id = map_frame_;
  raw_path_right_.header.frame_id = map_frame_;

  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/uwb_optimized/odom", 10);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/uwb_optimized/path", 10);
  pub_attitude_ = nh_.advertise<geometry_msgs::QuaternionStamped>("/uwb_optimized/attitude", 10);
  optimized_path_.header.frame_id = map_frame_;

  // 3. Configure ISAM2
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;  // More frequent relinearization
  params.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(params);

  last_velocity_ = gtsam::Vector3::Zero();

  // [NEW] Start timer for synchronized updates
  update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                   &UwbGtsamOptimizer::TimerCallback, this);

  ROS_INFO("Stereo UWB Optimizer Initialized in frame: %s", map_frame_.c_str());
  ROS_INFO("  - Update rate: %.1f Hz", update_rate_);
  ROS_INFO("  - Sync window: %.3f s", measurement_sync_window_);
  ROS_INFO("  - Person velocity sigma: %.2f m/s", person_vel_sigma_);
}

// ===========================================================================
// HELPER: Create robust noise model with FOV-based weighting
// ===========================================================================
gtsam::SharedNoiseModel UwbGtsamOptimizer::CreateRobustNoiseModel(double azimuth) {
  double fov_limit_rad = fov_limit_deg_ * M_PI / 180.0; 
  double fov_half_rad = fov_limit_rad / 2.0;            
  double abs_az = std::abs(azimuth);
  double ratio = std::min(abs_az / fov_half_rad, 1.0);

  // Smoother penalty function
  double penalty_factor = 1.0 + 4.0 * std::pow(ratio, 3);
  if (abs_az > fov_half_rad) penalty_factor = 50.0; 

  double current_az_sigma = (noise_angle_sigma_ * M_PI / 180.0) * penalty_factor;

  gtsam::Vector3 sigmas;
  sigmas << noise_range_sigma_,              
            current_az_sigma,                
            noise_elev_sigma_ * M_PI / 180.0;  
  
  auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
}

// ===========================================================================
// RAW CALLBACK - Now just stores measurements, doesn't update
// ===========================================================================
void UwbGtsamOptimizer::RawUwbCallback(
    const ros::MessageEvent<std_msgs::Int32MultiArray const>& event,
    const std::string& anchor_frame_id) {
  
  const std_msgs::Int32MultiArray::ConstPtr& msg = event.getMessage();
  ros::Time current_ros_time = event.getReceiptTime();
  double current_time = current_ros_time.toSec();

  // 1. Parse Data
  if (msg->data.size() < 3) return;
  double raw_r_cm  = msg->data[0];
  double raw_az_deg = -msg->data[1];  // Sign convention
  double raw_el_deg = msg->data[2];

  // 2. Coordinate Conversion
  double r_inst  = raw_r_cm / 100.0;
  double az_inst = raw_az_deg * M_PI / 180.0; 
  double el_inst = raw_el_deg * M_PI / 180.0;   

  // 3. Apply Low Pass Filter
  LpfState& state = lpf_states_[anchor_frame_id];
  double r_final, az_final, el_final; 

  if (!state.is_initialized) {
      state.range = r_inst; 
      state.azimuth = az_inst; 
      state.elevation = el_inst;
      state.is_initialized = true;
  } else {
      state.range     = lpf_gain_ * r_inst  + (1.0 - lpf_gain_) * state.range;
      state.azimuth   = lpf_gain_ * az_inst + (1.0 - lpf_gain_) * state.azimuth;
      state.elevation = lpf_gain_ * el_inst + (1.0 - lpf_gain_) * state.elevation;
  }
  r_final = state.range; 
  az_final = state.azimuth; 
  el_final = state.elevation;

  // 4. Lookup Sensor Pose
  tf::StampedTransform tf_map_sensor;
  try {
    tf_listener_.lookupTransform(map_frame_, anchor_frame_id, 
                                 ros::Time(0), tf_map_sensor);  // Use latest
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(2, "TF Error: %s", ex.what());
    return;
  }

  gtsam::Pose3 sensor_pose_global = gtsam::Pose3(
      gtsam::Rot3(tf_map_sensor.getRotation().w(), tf_map_sensor.getRotation().x(),
                  tf_map_sensor.getRotation().y(), tf_map_sensor.getRotation().z()),
      gtsam::Point3(tf_map_sensor.getOrigin().x(), tf_map_sensor.getOrigin().y(), 
                    tf_map_sensor.getOrigin().z())
  );

  // --- VISUALIZATION ---
  double lx = r_final * std::cos(az_final) * std::cos(el_final);
  double ly = r_final * std::sin(az_final) * std::cos(el_final);
  double lz = r_final * std::sin(el_final);
  gtsam::Point3 global_filtered_pt = sensor_pose_global.transformFrom(gtsam::Point3(lx, ly, lz));

  geometry_msgs::PoseStamped filtered_pose;
  filtered_pose.header.stamp = current_ros_time;
  filtered_pose.header.frame_id = map_frame_;
  filtered_pose.pose.position.x = global_filtered_pt.x();
  filtered_pose.pose.position.y = global_filtered_pt.y();
  filtered_pose.pose.position.z = global_filtered_pt.z();
  
  double sensor_yaw = tf::getYaw(tf_map_sensor.getRotation());
  tf::Quaternion q_vis;
  q_vis.setRPY(0, 0, sensor_yaw + az_final); 
  filtered_pose.pose.orientation.w = q_vis.w();
  filtered_pose.pose.orientation.x = q_vis.x();
  filtered_pose.pose.orientation.y = q_vis.y();
  filtered_pose.pose.orientation.z = q_vis.z();

  bool is_left = (anchor_frame_id.find("left") != std::string::npos);
  if (is_left) {
      raw_path_left_.poses.push_back(filtered_pose);
      if (raw_path_left_.poses.size() > 2000) raw_path_left_.poses.erase(raw_path_left_.poses.begin());
      raw_path_left_.header.stamp = current_ros_time;
      pub_raw_path_left_.publish(raw_path_left_);
  } else {
      raw_path_right_.poses.push_back(filtered_pose);
      if (raw_path_right_.poses.size() > 2000) raw_path_right_.poses.erase(raw_path_right_.poses.begin());
      raw_path_right_.header.stamp = current_ros_time;
      pub_raw_path_right_.publish(raw_path_right_);
  }

  // 5. [NEW] Store measurement for synchronized fusion
  {
    std::lock_guard<std::mutex> lock(measurement_mutex_);
    
    PendingMeasurement& pending = is_left ? pending_left_ : pending_right_;
    pending.sensor_pose = sensor_pose_global;
    pending.range = r_final;
    pending.azimuth = az_final;
    pending.elevation = el_final;
    pending.timestamp = current_time;
    pending.valid = true;
  }

  // 6. Initialize if needed
  if (!is_initialized_) {
    gtsam::Point3 global_guess(global_filtered_pt.x(), global_filtered_pt.y(), 0.0);
    InitializeSystem(current_time, global_guess);
  }
}

// ===========================================================================
// [NEW] TIMER CALLBACK - Synchronized fusion of both sensors
// ===========================================================================
void UwbGtsamOptimizer::TimerCallback(const ros::TimerEvent& event) {
  if (!is_initialized_) return;

  double current_time = event.current_real.toSec();
  double dt = current_time - last_time_;
  
  // Skip if too short
  if (dt < keyframe_time_thresh_) return;

  std::lock_guard<std::mutex> lock(measurement_mutex_);

  // Check if we have any valid measurements
  bool have_left = pending_left_.valid && 
                   (current_time - pending_left_.timestamp) < measurement_sync_window_;
  bool have_right = pending_right_.valid && 
                    (current_time - pending_right_.timestamp) < measurement_sync_window_;

  if (!have_left && !have_right) {
    return;
  }

  current_key_++;

  // --- 1. MOTION MODEL ---
  gtsam::Expression<gtsam::Pose3> x_prev(X(current_key_ - 1));
  gtsam::Expression<gtsam::Pose3> x_curr(X(current_key_));
  gtsam::Expression<gtsam::Vector3> v_prev(V(current_key_ - 1));

  gtsam::Expression<gtsam::Point3> pos_prev(positionFromPose, x_prev);
  gtsam::Expression<gtsam::Point3> pos_curr(positionFromPose, x_curr);
  gtsam::Expression<gtsam::Point3> pos_pred = pos_prev + (dt * v_prev);

  gtsam::Vector3 pos_noise_sigmas;
  pos_noise_sigmas << 0.05 + person_vel_sigma_ * dt, 
                      0.05 + person_vel_sigma_ * dt, 
                      0.02;
  auto pos_noise = gtsam::noiseModel::Diagonal::Sigmas(pos_noise_sigmas);
  
  // FIX: Create actual Point3 for the zero measurement
  gtsam::Point3 zero_point(0, 0, 0);
  graph_.add(gtsam::ExpressionFactor<gtsam::Point3>(
      pos_noise, zero_point, pos_curr - pos_pred));

  // Rotation stabilization
  gtsam::Vector6 pose_slack;
  pose_slack << 0.01, 0.01, 0.01, 100.0, 100.0, 100.0;
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(current_key_ - 1), X(current_key_), gtsam::Pose3(), 
      gtsam::noiseModel::Diagonal::Sigmas(pose_slack)));

  // --- 2. VELOCITY SMOOTHNESS ---
  double accel_limit = person_vel_sigma_;
  gtsam::Vector3 vel_sigmas;
  vel_sigmas << accel_limit * dt, accel_limit * dt, person_z_vel_sigma_ * dt;
  
  // FIX: Create actual Vector3 for zero delta
  gtsam::Vector3 zero_vec3 = gtsam::Vector3::Zero();
  graph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
      V(current_key_ - 1), V(current_key_), zero_vec3, 
      gtsam::noiseModel::Diagonal::Sigmas(vel_sigmas)));

  // --- 3. VELOCITY MAGNITUDE CONSTRAINT ---
  gtsam::Vector3 v_limit_sigmas;
  v_limit_sigmas << 2.0, 2.0, 0.1;
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(current_key_), zero_vec3, 
      gtsam::noiseModel::Diagonal::Sigmas(v_limit_sigmas)));

  // --- 4. ADD MEASUREMENT FACTORS ---
  int num_measurements = 0;

  if (have_left) {
    auto noise_left = CreateRobustNoiseModel(pending_left_.azimuth);
    graph_.add(UwbPdoaFactor(X(current_key_), 
                             pending_left_.sensor_pose,
                             pending_left_.range, 
                             pending_left_.azimuth, 
                             pending_left_.elevation, 
                             noise_left));
    num_measurements++;
    pending_left_.valid = false;
  }

  if (have_right) {
    auto noise_right = CreateRobustNoiseModel(pending_right_.azimuth);
    graph_.add(UwbPdoaFactor(X(current_key_), 
                             pending_right_.sensor_pose,
                             pending_right_.range, 
                             pending_right_.azimuth, 
                             pending_right_.elevation, 
                             noise_right));
    num_measurements++;
    pending_right_.valid = false;
  }

  ROS_DEBUG_THROTTLE(1.0, "Fusing %d measurements at key %ld", num_measurements, current_key_);

  // --- 5. INITIAL ESTIMATE ---
  gtsam::Vector3 pred_move = last_velocity_ * dt;
  gtsam::Pose3 predicted_pose = gtsam::Pose3(last_pose_.rotation(), 
                                              last_pose_.translation() + pred_move);
  
  initial_estimate_.insert(X(current_key_), predicted_pose);
  initial_estimate_.insert(V(current_key_), last_velocity_);

  // --- 6. OPTIMIZE ---
  try {
    isam_.update(graph_, initial_estimate_);
    isam_.update();

    gtsam::Values result = isam_.calculateEstimate();
    
    last_pose_ = result.at<gtsam::Pose3>(X(current_key_));
    last_velocity_ = result.at<gtsam::Vector3>(V(current_key_));

    last_velocity_(2) = std::max(-0.3, std::min(0.3, last_velocity_(2)));

  } catch (const gtsam::IndeterminantLinearSystemException& e) {
    ROS_ERROR("GTSAM Error: %s", e.what());
    current_key_--;
    graph_.resize(0);
    initial_estimate_.clear();
    return;
  }

  graph_.resize(0);
  initial_estimate_.clear();
  last_time_ = current_time;

  CheckAndPublish(current_time);
}

// ===========================================================================
// LEGACY: PerformPersonUpdate (kept for compatibility, redirects to timer logic)
// ===========================================================================
void UwbGtsamOptimizer::PerformPersonUpdate(double dt) {
  // This is now handled by TimerCallback
  // Keeping method signature for compatibility
}

// ===========================================================================
// LEGACY METHODS (Unchanged)
// ===========================================================================

gtsam::SharedNoiseModel UwbGtsamOptimizer::GetDynamicNoiseModel(
    const gtsam::Point3& local_point,
    const tf::StampedTransform& tf_map_anchor) {
  
  double range = local_point.norm();
  double azimuth = std::atan2(local_point.y(), local_point.x());
  double angle_deg = std::abs(azimuth) * 180.0 / M_PI;

  if (angle_deg > fov_limit_deg_) return nullptr;

  double ratio = angle_deg / fov_limit_deg_;
  double edge_penalty = 1.0 + 20.0 * std::pow(ratio, 6);

  double var_range = std::pow(noise_range_sigma_, 2);
  double sigma_angle_rad = (noise_angle_sigma_ * M_PI / 180.0) * edge_penalty;
  double var_lateral = std::pow(range * std::sin(sigma_angle_rad), 2);
  double var_elevation = std::pow(1.0, 2);

  gtsam::Matrix3 local_cov = gtsam::Matrix3::Zero();
  local_cov(0, 0) = var_range;
  local_cov(1, 1) = var_lateral;
  local_cov(2, 2) = var_elevation;

  tf::Matrix3x3 tf_rot = tf_map_anchor.getBasis();
  Eigen::Matrix3d rot_eigen;
  tf::matrixTFToEigen(tf_rot, rot_eigen);

  gtsam::Matrix3 global_cov = rot_eigen * local_cov * rot_eigen.transpose();

  return gtsam::noiseModel::Gaussian::Covariance(global_cov);
}

void UwbGtsamOptimizer::UwbCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg,
    const std::string& anchor_frame_id) {
  
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

  auto noise_model = GetDynamicNoiseModel(local_meas, tf_map_anchor);
  if (!noise_model) return;

  tf::Vector3 local_vec(local_meas.x(), local_meas.y(), local_meas.z());
  tf::Vector3 global_vec = tf_map_anchor * local_vec;
  gtsam::Point3 global_meas(global_vec.x(), global_vec.y(), global_vec.z());

  if (!is_initialized_) {
    InitializeSystem(current_time, global_meas);
  } else {
    double dt = current_time - last_time_;
    if (dt < 0.005) return;
    PerformUpdate(dt, global_meas, noise_model);
  }
  
  last_time_ = current_time; 
}

void UwbGtsamOptimizer::SimplePathCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Legacy - not used
}

void UwbGtsamOptimizer::InitializeSystem(double time, const gtsam::Point3& initial_pos) {
  last_pose_ = gtsam::Pose3(gtsam::Rot3(), initial_pos);
  last_velocity_ = gtsam::Vector3::Zero();
  
  // FIX: Create actual Vector3, not lazy expression
  gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
  
  initial_estimate_.insert(X(0), last_pose_);
  initial_estimate_.insert(V(0), zero_velocity);  // Use the actual vector

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), last_pose_, gtsam::noiseModel::Isotropic::Sigma(6, 0.5)));
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(0), zero_velocity,  // Use the actual vector
      gtsam::noiseModel::Isotropic::Sigma(3, 0.5)));

  isam_.update(graph_, initial_estimate_);
  
  graph_.resize(0);
  initial_estimate_.clear();

  last_time_ = time;  // FIX: was "last_time"
  is_initialized_ = true;
  ROS_INFO("System Initialized at (%.2f, %.2f, %.2f)", 
           initial_pos.x(), initial_pos.y(), initial_pos.z());
}

void UwbGtsamOptimizer::PerformUpdate(double dt, const gtsam::Point3& meas_global,
                                      const gtsam::SharedNoiseModel& noise_model) {
  // Legacy method - kept for PoseStamped callback
  double time_from_last = (last_time_ + dt) - last_keyframe_time_;
  
  if (time_from_last < keyframe_time_thresh_) {
    return;
  }
  
  last_keyframe_pos_ = meas_global;
  last_keyframe_time_ = last_time_ + dt;
  current_key_++;

  double pos_noise = 1.0 * dt; 
  double vel_noise = 1.0 * dt; 
  gtsam::Vector3 vel_sigmas(vel_noise, vel_noise, z_velocity_sigma_ * dt);
  graph_.add(gtsam::BetweenFactor<gtsam::Vector3>(
      V(current_key_ - 1), V(current_key_), gtsam::Vector3::Zero(),
      gtsam::noiseModel::Diagonal::Sigmas(vel_sigmas)));

  gtsam::Vector3 pred_move = last_velocity_ * dt;
  gtsam::Pose3 odom_delta = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(pred_move));

  gtsam::Vector6 pose_sigmas;
  pose_sigmas << 500.0, 500.0, 500.0, pos_noise, pos_noise, z_acceleration_sigma_;
  auto pred_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas);

  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(current_key_ - 1), X(current_key_), odom_delta, pred_noise));

  gtsam::Matrix6 full_cov = gtsam::Matrix6::Zero();
  full_cov.block<3, 3>(0, 0) = gtsam::Matrix3::Identity() * 500.0;
  
  auto gaussian_noise = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise_model);
  if (gaussian_noise) {
    full_cov.block<3, 3>(3, 3) = gaussian_noise->covariance();
  } else {
    full_cov.block<3, 3>(3, 3) = gtsam::Matrix3::Identity() * 0.5;
  }

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(current_key_), gtsam::Pose3(gtsam::Rot3(), meas_global),
      gtsam::noiseModel::Gaussian::Covariance(full_cov)));

  initial_estimate_.insert(X(current_key_), last_pose_.compose(odom_delta));
  initial_estimate_.insert(V(current_key_), last_velocity_);

  isam_.update(graph_, initial_estimate_);
  isam_.update();
  
  graph_.resize(0);
  initial_estimate_.clear();

  gtsam::Values result = isam_.calculateEstimate();
  last_pose_ = result.at<gtsam::Pose3>(X(current_key_));
  last_velocity_ = result.at<gtsam::Vector3>(V(current_key_));

  if (std::abs(last_velocity_(2)) > 0.1) {
    last_velocity_(2) = std::copysign(0.3, last_velocity_(2));
  }

  CheckAndPublish(last_time_ + dt);
}

void UwbGtsamOptimizer::CheckAndPublish(double time_stamp) {
  try {
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
    if (optimized_path_.poses.size() > 5000) {
      optimized_path_.poses.erase(optimized_path_.poses.begin());
    }
    optimized_path_.header.stamp = ros::Time(time_stamp);
    pub_path_.publish(optimized_path_);

    geometry_msgs::QuaternionStamped attitude_msg;
    attitude_msg.header.stamp = ros::Time(time_stamp);
    attitude_msg.header.frame_id = map_frame_;
    attitude_msg.quaternion.w = q.w();
    attitude_msg.quaternion.x = q.x();
    attitude_msg.quaternion.y = q.y();
    attitude_msg.quaternion.z = q.z();
    pub_attitude_.publish(attitude_msg);
    
    nav_msgs::Odometry odom;
    odom.header = pose_stamped.header;
    odom.child_frame_id = "tracked_person";
    odom.pose.pose = pose_stamped.pose;
    odom.twist.twist.linear.x = last_velocity_.x();
    odom.twist.twist.linear.y = last_velocity_.y();
    odom.twist.twist.linear.z = last_velocity_.z();
    pub_odom_.publish(odom);

  } catch (gtsam::IndeterminantLinearSystemException& e) {
    ROS_ERROR("GTSAM Error: %s", e.what());
  }
}

} // namespace uwb_fusion