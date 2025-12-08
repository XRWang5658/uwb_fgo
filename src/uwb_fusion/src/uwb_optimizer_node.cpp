#include <ros/ros.h>
#include "uwb_fusion/uwb_gtsam_optimizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "uwb_gtsam_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  uwb_fusion::UwbGtsamOptimizer optimizer(nh, nh_private);

  ros::spin();
  return 0;
}