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
#include <geometry_msgs//Twist.h>
#include <sensor_msgs/Joy.h>

#include <plan_manage/planner_manager.h>

using std::vector;
using std::string;

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
	Eigen::Vector4d yaw_quaternion_;
	double yaw_angle_;
	double v_min_, v_max_;
    string odom_topic_;

	/* visualization setting */
	double line_width_;
	std::vector<Eigen::Vector2d> history_pos_, history_vel_;

	Eigen::Vector2d odom_pos_, odom_vel_;  // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector2d start_pt_;
    Eigen::Vector3d start_vel_, start_acc_, start_yaw_;  // start state
    Eigen::Vector2d end_pt_;                              // target state
  
  	/* ROS utils */
	ros::NodeHandle node_;
	ros::Timer cmd_timer_, low_mpc_timer_, high_mpcc_timer_, safety_check_timer_;
	ros::Subscriber trigger_sub_, goal_sub_, odom_sub_, joy_sub_;
	ros::Publisher pos_cmd_pub_, pos_cmd_vis_pub_, exected_traj_pub_, path_pub_;

	/* ROS Timer Function */
	void cmdCallback(const ros::TimerEvent& e);
	void lowMpcCallback(const ros::TimerEvent& e);
	void highMpccCallback(const ros::TimerEvent& e);
	void trajSafetyCallback(const ros::TimerEvent& e);

	/* ROS functions */
	void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void publishPathMessage(const std::vector<Eigen::Vector2d>& path);
    void publishPathMessage(const std::vector<Eigen::Vector3d>& path);

	/* helper functions */
	bool callLowMpc();
	bool callHighMpcc(const Eigen::Vector3d& start_state);
    static geometry_msgs::PoseStamped getRos3DPoseFromAstar2DLocation(double x, double y);

public:
	AdaptiveReplanFsm(){};
	~AdaptiveReplanFsm(){};

	void init(ros::NodeHandle& nh);
	
	// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace adaptive_planner

#endif