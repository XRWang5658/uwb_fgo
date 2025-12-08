#ifndef UWB_FUSION_UWB_GTSAM_OPTIMIZER_H_
#define UWB_FUSION_UWB_GTSAM_OPTIMIZER_H_

#include <ros/ros.h>
#include <ros/message_event.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32MultiArray.h> 

#include <string>
#include <memory>
#include <map>

// GTSAM Includes
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uwb_fusion/uwb_pdoa_factor.h"

namespace uwb_fusion {

// Low Pass Filter State
struct LpfState {
    double range = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    bool is_initialized = false;
};

class UwbGtsamOptimizer {
 public:
  UwbGtsamOptimizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~UwbGtsamOptimizer() = default;

 private:
  // --- ROS Components ---
  ros::NodeHandle nh_;
  
  ros::Subscriber sub_raw_left_;
  ros::Subscriber sub_raw_right_;

  ros::Subscriber sub_uwb_left_;
  ros::Subscriber sub_uwb_right_;
  ros::Subscriber sub_simple_; 

  ros::Publisher pub_odom_;
  tf::TransformListener tf_listener_;
  ros::Publisher pub_path_;
  ros::Publisher pub_attitude_;
  nav_msgs::Path optimized_path_;
  
  // [NEW] Separate Publishers for Visualization
  ros::Publisher pub_raw_path_left_;      
  ros::Publisher pub_raw_path_right_;
  
  // [NEW] Separate Path Containers
  nav_msgs::Path raw_path_left_;          
  nav_msgs::Path raw_path_right_;
  
  ros::Publisher pub_raw_attitude_; 

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
  double fov_limit_deg_;     
  double noise_range_sigma_; 
  double noise_angle_sigma_; 
  double blind_cov_thresh_;  
  std::string map_frame_;    

  double noise_elev_sigma_;    
  double person_vel_sigma_;    
  double person_z_vel_sigma_;  

  // Low Pass Filter
  double lpf_gain_; 
  std::map<std::string, LpfState> lpf_states_;

  // Keyframe selection parameters
  double keyframe_distance_thresh_;  
  double keyframe_time_thresh_;      
  gtsam::Point3 last_keyframe_pos_;  
  double last_keyframe_time_;        

  double z_velocity_sigma_;       
  double z_acceleration_sigma_;   

  // --- Methods ---
  void UwbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                   const std::string& anchor_frame_id);

  void SimplePathCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  void RawUwbCallback(const ros::MessageEvent<std_msgs::Int32MultiArray const>& event,
                      const std::string& anchor_frame_id);

  void PerformPersonUpdate(double dt);

  gtsam::SharedNoiseModel GetDynamicNoiseModel(
      const gtsam::Point3& local_point,
      const tf::StampedTransform& tf_map_anchor);

  void PerformUpdate(double dt, const gtsam::Point3& meas_global,
                     const gtsam::SharedNoiseModel& noise_model);

  void InitializeSystem(double time, const gtsam::Point3& initial_pos);

  void CheckAndPublish(double time_stamp);
};

}  // namespace uwb_fusion

#endif  // UWB_FUSION_UWB_GTSAM_OPTIMIZER_H_