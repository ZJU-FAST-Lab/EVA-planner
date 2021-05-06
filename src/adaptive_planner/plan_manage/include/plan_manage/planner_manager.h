#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <ros/ros.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <path_searching/astar.h>
#include <path_searching/low_mpc.h>
#include <path_searching/high_mpcc_optimizer.h>

namespace adaptive_planner{

class AdaptivePlannerManager
{
public:
	AdaptivePlannerManager();
	~AdaptivePlannerManager();

	/* main planning interface */
	double safety_dist_;
	bool LowMpc(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
	bool HighMpcc(Eigen::Matrix3d start_state, bool if_adaptive);
	void resetMPCCinitial();
	bool safeCheck();

	void initPlanModules(ros::NodeHandle& nh);

	std::vector<Eigen::Vector3d> global_path_ , local_path_, visited_nodes_, low_mpc_traj_;
	std::vector<std::vector<Eigen::Vector3d>> high_mpcc_traj_;
	std::vector<Eigen::Vector3d> high_mpcc_traj_pos_, high_mpcc_traj_vel_, high_mpcc_traj_acc_, high_mpcc_traj_ref_;
	Eigen::Vector3d  			 high_mpcc_theta_next_;
	Eigen::Vector3d              local_goal_;

private:
  	/* main planning algorithms & modules */
  	SDFMap::Ptr sdf_map_;
	EDTEnvironment::Ptr edt_environment_;
	unique_ptr<Astar> path_finder_;
	unique_ptr<low_mpc_planner> low_mpc_planner_;
	unique_ptr<high_mpcc_optimizer> high_mpcc_optimizer_;


public:
  	typedef unique_ptr<AdaptivePlannerManager> Ptr;
};

}	// namespace adaptive_planner

#endif