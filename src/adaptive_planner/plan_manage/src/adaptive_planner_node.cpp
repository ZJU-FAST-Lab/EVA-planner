#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/adaptive_replan_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace adaptive_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "adaptive_planner_node");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  AdaptiveReplanFsm adaptive_replan;
  adaptive_replan.init(nh);

  //ros::Duration(1.0).sleep();
  //ros::spin();
  
  ros::waitForShutdown();

  return 0;
}
