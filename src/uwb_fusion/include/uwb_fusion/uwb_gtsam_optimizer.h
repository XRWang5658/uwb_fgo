#ifndef UWB_FUSION_UWB_GTSAM_OPTIMIZER_H_
#define UWB_FUSION_UWB_GTSAM_OPTIMIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <string>
#include <memory>

// GTSAM Includes
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace uwb_fusion {

class UwbGtsamOptimizer {
 public:
  UwbGtsamOptimizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~UwbGtsamOptimizer() = default;

 private:
  // --- ROS Components ---
  ros::NodeHandle nh_;
  ros::Subscriber sub_uwb_left_;
  ros::Subscriber sub_uwb_right_;
  ros::Subscriber sub_simple_; 
  ros::Publisher pub_odom_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_path_;
  ros::Publisher pub_attitude_;
  nav_msgs::Path optimized_path_;
  ros::Publisher pub_raw_path_;      // Publisher for the raw UWB path
  ros::Publisher pub_raw_attitude_; // Publisher for the raw UWB attitude
  nav_msgs::Path raw_path_;          // Container for the raw UWB path

  // --- GTSAM Components ---
  gtsam::ISAM2 isam_;
  gtsam::Values initial_estimate_;
  gtsam::NonlinearFactorGraph graph_;

  // --- State Variables ---
  long current_key_;
  double last_time_;
  bool is_initialized_;
  gtsam::Pose3 last_pose_;
  gtsam::Vector3 last_velocity_;

  // --- Parameters ---
  double fov_limit_deg_;     // Hard cut-off for Field of View
  double noise_range_sigma_; // Range accuracy (m)
  double noise_angle_sigma_; // Angular accuracy (deg)
  double blind_cov_thresh_;  // Safety threshold for covariance trace
  std::string map_frame_;    // The tracking frame (e.g. "odom")

  // Keyframe selection parameters
  double keyframe_distance_thresh_;  // Minimum distance to add keyframe (meters)
  double keyframe_time_thresh_;      // Minimum time to add keyframe (seconds)
  gtsam::Point3 last_keyframe_pos_;  // Last keyframe position
  double last_keyframe_time_;        // Last keyframe timestamp

  double z_velocity_sigma_;       // Soft constraint on z velocity
  double z_acceleration_sigma_;   // Soft constraint on z acceleration

  // --- Methods ---

  /**
   * @brief Callback for UWB anchors mounted on the robot.
   */
  void UwbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                   const std::string& anchor_frame_id);

  /**
   * @brief Simple XYZ callback for testing single-input performance.
   */
  void SimplePathCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief Scientifically constructs the noise model based on geometry.
   * Handles "Stereo UWB" logic: Range is trusted, Angle degrades with distance/edge.
   */
  gtsam::SharedNoiseModel GetDynamicNoiseModel(
      const gtsam::Point3& local_point,
      const tf::StampedTransform& tf_map_anchor);

  /**
   * @brief Core update loop: Adds factors and runs ISAM2.
   */
  void PerformUpdate(double dt, const gtsam::Point3& meas_global,
                     const gtsam::SharedNoiseModel& noise_model);

  void InitializeSystem(double time, const gtsam::Point3& initial_pos);

  void CheckAndPublish(double time_stamp);
};

}  // namespace uwb_fusion

#endif  // UWB_FUSION_UWB_GTSAM_OPTIMIZER_H_