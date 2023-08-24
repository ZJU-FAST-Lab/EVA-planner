
#include <ros/duration.h>
#include <ros/ros.h>
#include <string>

#include "erl_costmap_ros/erl_costmap_ros.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "erl_costmap_ros");
  ros::NodeHandle nh("~");

  double note_freq = 100.0;
  // set node frequency by reading parameter server otherwise set it to default 100.0
  if (!nh.getParam("note_freq", note_freq)) {
    ROS_WARN("[erl_costmap_node] could not find [note_freq] parameter, use default freq %.2f", note_freq);
  } else {
    ROS_WARN("[erl_costmap_node] get note_freq: %.2f", note_freq);
  }
  ros::Rate rate(note_freq);

  ErlCostmapROS costmap_node(nh);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
}
