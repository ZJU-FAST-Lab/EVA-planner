#include <ros/ros.h>
#include <plan_manage/adaptive_replan_fsm.h>

using namespace adaptive_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "adaptive_planner_node");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(18);
  spinner.start();

  AdaptiveReplanFsm adaptive_replan;
  adaptive_replan.init(nh);
  
  ros::waitForShutdown();

  return 0;
}
