#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class BaselineVisualizer {
 public:
  BaselineVisualizer() {
    ros::NodeHandle nh;
    sub_baseline_pose_ = nh.subscribe("/uwb_pose", 10, &BaselineVisualizer::PoseCallback, this);
    pub_baseline_path_ = nh.advertise<nav_msgs::Path>("/uwb/baseline_path", 10);

    baseline_path_.header.frame_id = "map";  // Set the frame ID
  }

 private:
  ros::Subscriber sub_baseline_pose_;
  ros::Publisher pub_baseline_path_;
  nav_msgs::Path baseline_path_;

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Transform the position from FRU to FLU
    geometry_msgs::PoseStamped transformed_pose = *msg;
    transformed_pose.pose.position.y = -msg->pose.position.y;

    // Transform the orientation from FRU to FLU
    tf2::Quaternion q_fru(msg->pose.orientation.x,
                          msg->pose.orientation.y,
                          msg->pose.orientation.z,
                          msg->pose.orientation.w);

    // Apply a 180° rotation around the Z-axis to convert FRU to FLU
    tf2::Quaternion q_flu;
    tf2::Quaternion rotation_180(0, 0, 1, 0);  // 180° rotation around Z-axis
    q_flu = rotation_180 * q_fru;
    q_flu.normalize();

    transformed_pose.pose.orientation.x = q_flu.x();
    transformed_pose.pose.orientation.y = q_flu.y();
    transformed_pose.pose.orientation.z = q_flu.z();
    transformed_pose.pose.orientation.w = q_flu.w();

    // Add the transformed pose to the path
    baseline_path_.header.stamp = transformed_pose.header.stamp;
    baseline_path_.poses.push_back(transformed_pose);

    // Limit the path size to avoid memory issues
    if (baseline_path_.poses.size() > 100) {
      baseline_path_.poses.erase(baseline_path_.poses.begin());
    }

    // Publish the path
    pub_baseline_path_.publish(baseline_path_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "baseline_visualizer");
  BaselineVisualizer visualizer;
  ros::spin();
  return 0;
}