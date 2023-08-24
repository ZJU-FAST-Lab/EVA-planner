#include <plan_manage/planner_manager.h>
#include <memory>
#include <thread>

namespace adaptive_planner{

AdaptivePlannerManager::AdaptivePlannerManager(){}

AdaptivePlannerManager::~AdaptivePlannerManager(){ std::cout << "des manager" << std::endl; }

void AdaptivePlannerManager::initPlanModules(ros::NodeHandle& nh){
    nh.param("fsm/safety_dist",   safety_dist_, 0.1);
    // init map
//    sdf_map_.reset(new SDFMap);
    sdf_map_ = std::make_unique<SDFMap>();
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setParam(nh);
    edt_environment_->setMap(sdf_map_);

    // init path searching
    path_finder_ = std::make_unique<Astar>();
    path_finder_->setParam(nh);
    path_finder_->setEnvironment(sdf_map_);
    path_finder_->init();

    low_mpc_planner_ = std::make_unique<low_mpc_planner>();
    low_mpc_planner_->init(nh);
    low_mpc_planner_->setEnvironment(edt_environment_);

    high_mpcc_optimizer_ = std::make_unique<high_mpcc_optimizer>();
    high_mpcc_optimizer_->setParam(nh);
    high_mpcc_optimizer_->setEnvironment(edt_environment_);
}

bool AdaptivePlannerManager::LowMpc(Eigen::Vector2d& start_pt, Eigen::Vector2d& end_pt){

    // global path search
    int status = path_finder_->search(start_pt, end_pt);
    if (status == 1){
        // get the local normalized path
        ros::Time t_s = ros::Time::now();
        global_path_ = path_finder_->getPath();
        local_path_  = path_finder_->getLocalPath(global_path_);

        // low mpc optimization
        low_mpc_traj_  = low_mpc_planner_->lowMpcOptimization(start_pt, local_path_);
        return true;
    } else{
        return false;
    }
}

void AdaptivePlannerManager::resetMPCCinitial(){
    high_mpcc_optimizer_->resetInputInital();
}

bool AdaptivePlannerManager::HighMpcc(const Eigen::Vector3d& start_state, bool if_adaptive){
    // high mpcc optimization
    high_mpcc_traj_         = high_mpcc_optimizer_->mpccOptimizeTraj(start_state, low_mpc_traj_, if_adaptive);

    high_mpcc_traj_pos_     = high_mpcc_traj_[0];
    high_mpcc_traj_vel_     = high_mpcc_traj_[1];
    high_mpcc_traj_ref_     = high_mpcc_traj_[2];
    return true;
}

bool AdaptivePlannerManager::safeCheck(){

    auto num = static_cast<int>(low_mpc_traj_.size());

    for (int i=0; i<num; i++){
      int isOccupied = sdf_map_->getInflateOccupancy(low_mpc_traj_[i]);
      if (1 == isOccupied){
        return false;
      }
    }
    return true;
}

}	// namespace adaptive_planner