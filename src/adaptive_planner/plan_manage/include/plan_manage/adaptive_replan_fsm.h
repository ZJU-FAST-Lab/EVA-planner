#ifndef ADAPTIVE_REPLAN_FSM
#define ADAPTIVE_REPLAN_FSM

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "quadrotor_msgs/PositionCommand.h"

#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <sensor_msgs/Joy.h>
using std::vector;

namespace adaptive_planner{
// This class is the Finite Systems Machine of adaptive replanner, 
// which charges the several states in the planning process.
class AdaptiveReplanFsm
{
private:
	/* ---------- flag ---------- */
	enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_PATH, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  	enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };
	int flight_type_;

  	/* planning utils */
	AdaptivePlannerManager::Ptr planner_manager_;
	PlanningVisualization::Ptr visualization_;

	/* planning data */
	bool have_trigger_, have_target_, have_odom_, have_traj_, have_low_traj_, near_goal_;
  	FSM_EXEC_STATE exec_state_;
	double mpc_delta_T_;		// the delta T of each horizon in mpc
	double mpcc_opt_;			// optimization time of high mpcc
	ros::Time tMpc1_ , tMpc2_, tMpc1_lst_;	// tMpc1_: the moment that generate mpcc,	tMpc2_: the moment that prapare to generate mpcc
	bool if_adaptive_;
	std::vector<Eigen::Vector3d> high_mpcc_traj_pos_, high_mpcc_traj_vel_, high_mpcc_traj_acc_;
	double T_max_lowmpc_, T_max_highmpcc_; 
	bool show_lowmpc_time_, show_highmpcc_time_;
	Eigen::Vector2d yaw_0_;
	double yaw_angle_;
	int waypoint_num_, waypoint_flag_;
	double waypoints_[50][3];
	double v_min_, v_max_;

	/* visualization setting */
	double line_width_;
	std::vector<Eigen::Vector3d> history_pos_, history_vel_;

	Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  	Eigen::Quaterniond odom_orient_;

  	Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  	Eigen::Vector3d end_pt_;                              // target state
  
  	/* ROS utils */
	ros::NodeHandle node_;
	ros::Timer cmd_timer_, low_mpc_timer_, high_mpcc_timer_, safety_check_timer_;
	ros::Subscriber trigger_sub_, waypoint_sub_, odom_sub_, joy_sub_;
	ros::Publisher pos_cmd_pub_, pos_cmd_vis_pub_, exected_traj_pub_;

	/* ROS Timer Function */
	void cmdCallback(const ros::TimerEvent& e);
	void lowMpcCallback(const ros::TimerEvent& e);
	void highMpccCallback(const ros::TimerEvent& e);
	void trajSafetyCallback(const ros::TimerEvent& e);

	/* ROS functions */
	void waypointCallback(const nav_msgs::PathConstPtr& msg);
	void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  	void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
	void draw_history_traj();

	/* helper functions */
	bool callLowMpc();
	bool callHighMpcc(Eigen::Matrix3d start_state);

public:
	AdaptiveReplanFsm(){};
	~AdaptiveReplanFsm(){};

	void init(ros::NodeHandle& nh);
	
	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace adaptive_planner

#endif