#include <plan_manage/adaptive_replan_fsm.h>

namespace adaptive_planner{
    
void AdaptiveReplanFsm::init(ros::NodeHandle& nh){
    exec_state_    = FSM_EXEC_STATE::INIT;
    have_target_   = false;
    have_odom_     = false;
    have_traj_     = false;
    have_low_traj_ = false;
    near_goal_     = false;
    have_trigger_  = false;

    nh.param<std::string>("odom_topic",         odom_topic_, "/gazebo_p3d/odom");
    nh.param("multi_mpcc/Ts",                   mpc_delta_T_, 0.05);
    nh.param("multi_mpcc/Mpcc_opt",             mpcc_opt_, 0.01);
    nh.param("optimization/vel_upper",          v_max_,2.0);
    nh.param("optimization/vel_lower",          v_min_,-2.0);
    nh.param("multi_mpcc/show_lowmpc_time",     show_lowmpc_time_, false);
    nh.param("multi_mpcc/show_highmpcc_time",   show_highmpcc_time_, false);

    /* initialize main modules */
    planner_manager_ = std::make_unique<AdaptivePlannerManager>();
    planner_manager_->initPlanModules(nh);

    /* callback */
//    cmd_timer_          = nh.createTimer(ros::Duration(0.01), &AdaptiveReplanFsm::cmdCallback, this);
    low_mpc_timer_      = nh.createTimer(ros::Duration(2.0), &AdaptiveReplanFsm::lowMpcCallback, this);
    high_mpcc_timer_    = nh.createTimer(ros::Duration(mpc_delta_T_), &AdaptiveReplanFsm::highMpccCallback, this);
    safety_check_timer_ = nh.createTimer(ros::Duration(0.05), &AdaptiveReplanFsm::trajSafetyCallback, this);

    odom_sub_           = nh.subscribe(odom_topic_, 1, &AdaptiveReplanFsm::odometryCallback, this);
    goal_sub_           = nh.subscribe("/move_base_simple/goal", 1, &AdaptiveReplanFsm::goalCallback, this);
    path_pub_           = nh.advertise<nav_msgs::Path>("/low_mpc/path", 1);
    pos_cmd_pub_        = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    if_adaptive_  = true;
    T_max_lowmpc_ = T_max_highmpcc_ = 0.0;
}

void AdaptiveReplanFsm::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    // -------------- Manual set goal callback --------------
    // manual target type
    end_pt_ << msg->pose.position.x, msg->pose.position.y;

    cout << "Get waypoint!" << endl;
    have_target_ = true;
    near_goal_   = false;
}

void AdaptiveReplanFsm::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Quaterniond body_q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                   msg->pose.pose.orientation.x,
                                                   msg->pose.pose.orientation.y,
                                                   msg->pose.pose.orientation.z);

    Eigen::Matrix3d rotation_mat = body_q.toRotationMatrix();

    yaw_angle_ = atan2(rotation_mat(1, 0), rotation_mat(0, 0));

    // record history pos and vel
    if (have_target_){
        history_pos_.push_back(odom_pos_);
        history_vel_.push_back(odom_vel_);
    }

    have_odom_ = true;
}

//void AdaptiveReplanFsm::cmdCallback(const ros::TimerEvent& e){
//    if (have_traj_)
//    {
//        geometry_msgs::Point tmpPos;
//        geometry_msgs::Vector3 tmpVel, tmpAcc;
//        quadrotor_msgs::PositionCommand cmdMsg;
//        cmdMsg.header.frame_id = "world";
//        cmdMsg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
//        double t;
//        int k;
//        Eigen::Vector2d pos_L, pos_R, vel_L, vel_R, acc_L, acc_R,pos_yaw;
//        if (high_mpcc_traj_pos_.empty()){
//            t = (ros::Time::now() - tMpc1_).toSec();
//            k = floor(t/mpc_delta_T_);
////             cout <<"[0]  k:  "<< k << "  t:  " << t << endl;
//            acc_L = planner_manager_->high_mpcc_traj_acc_[k];
//            acc_R = planner_manager_->high_mpcc_traj_acc_[k+1];
//            vel_L = planner_manager_->high_mpcc_traj_vel_[k];
//            vel_R = planner_manager_->high_mpcc_traj_vel_[k+1];
//            pos_L = planner_manager_->high_mpcc_traj_pos_[k];
//            pos_R = planner_manager_->high_mpcc_traj_pos_[k+1];
//        }else if ((ros::Time::now()-tMpc2_).toSec() - mpcc_opt_ < 0  ){
//            t = (ros::Time::now() - tMpc1_lst_).toSec();
//            k = floor(t/mpc_delta_T_);
////             cout <<"[1]  k: " << k << "  t:  " << t << endl;
//            acc_L = high_mpcc_traj_acc_[k];
//            acc_R = high_mpcc_traj_acc_[k+1];
//            vel_L = high_mpcc_traj_vel_[k];
//            vel_R = high_mpcc_traj_vel_[k+1];
//            pos_L = high_mpcc_traj_pos_[k];
//            pos_R = high_mpcc_traj_pos_[k+1];
//        }else {
//            t = (ros::Time::now() - tMpc1_).toSec();
//            k = floor(t/mpc_delta_T_);
////             cout <<"[2]  k: " << k << "  t:  " << t << endl;
//            acc_L = planner_manager_->high_mpcc_traj_acc_[k];
//            acc_R = planner_manager_->high_mpcc_traj_acc_[k+1];
//            vel_L = planner_manager_->high_mpcc_traj_vel_[k];
//            vel_R = planner_manager_->high_mpcc_traj_vel_[k+1];
//            pos_L = planner_manager_->high_mpcc_traj_pos_[k];
//            pos_R = planner_manager_->high_mpcc_traj_pos_[k+1];
//        }
//        t = t - k * mpc_delta_T_;
//        tmpAcc.x = acc_L(0) + (acc_R(0) - acc_L(0)) * t / mpc_delta_T_;
//        tmpAcc.y = acc_L(1) + (acc_R(1) - acc_L(1)) * t / mpc_delta_T_;
//        tmpAcc.z = acc_L(2) + (acc_R(2) - acc_L(2)) * t / mpc_delta_T_;
//        cmdMsg.acceleration = tmpAcc;
//        tmpVel.x = vel_L(0) + acc_L(0) * t + (acc_R(0) - acc_L(0)) * t * t / mpc_delta_T_ / 2;
//        tmpVel.y = vel_L(1) + acc_L(1) * t + (acc_R(1) - acc_L(1)) * t * t / mpc_delta_T_ / 2;
//        tmpVel.z = vel_L(2) + acc_L(2) * t + (acc_R(2) - acc_L(2)) * t * t / mpc_delta_T_ / 2;
//        cmdMsg.velocity = tmpVel;
//        tmpPos.x = pos_L(0) + vel_L(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
//        tmpPos.y = pos_L(1) + vel_L(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
//        tmpPos.z = pos_L(2) + vel_L(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;
//        cmdMsg.position = tmpPos;
//
//        // yaw
//        Eigen::Vector2d temp_vec;
//        temp_vec << tmpVel.x, tmpVel.y;
//        double yaw_angle_cos = yaw_0_.dot(temp_vec) / yaw_0_.norm() / temp_vec.norm();
//        double yaw_angle;
//        if (temp_vec(1) < 0) {
//            yaw_angle = -acos(yaw_angle_cos);
//        } else{
//            yaw_angle = acos(yaw_angle_cos);
//        }
//        yaw_angle_ = yaw_angle;
//        Eigen::Vector2d temp_acc;
//        temp_acc << tmpAcc.x, tmpAcc.y;
//        double yaw_dot;
//        if (tmpVel.x * tmpAcc.y >= tmpVel.y * tmpAcc.x){
//            yaw_dot = temp_acc.norm();
//        } else{
//            yaw_dot = -temp_acc.norm();
//        }
//        cmdMsg.yaw = yaw_angle;
//        cmdMsg.yaw_dot = yaw_dot;
//
//        // draw cmd
//        Eigen::Vector3d pos,vel,acc;
//        pos << tmpPos.x, tmpPos.y, tmpPos.z;
//        vel << tmpVel.x, tmpVel.y, tmpVel.z;
//        acc << tmpAcc.x, tmpAcc.y, tmpAcc.z;
//        drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
//        drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
//        Eigen::Vector3d dir(cos(yaw_angle), sin(yaw_angle), 0.0);
//        drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
//
//        // draw exected_traj with vel color
//        // ADD
//
//        cmdMsg.header.stamp = ros::Time::now();
//        pos_cmd_pub_.publish(cmdMsg);
//    }
//    if (near_goal_){
//        geometry_msgs::Point tmpPos;
//        geometry_msgs::Vector3 tmpVel, tmpAcc;
//        quadrotor_msgs::PositionCommand cmdMsg;
//        cmdMsg.header.frame_id = "world";
//        cmdMsg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
//        tmpAcc.x = tmpAcc.y = tmpAcc.z = 0;
//        cmdMsg.acceleration = tmpAcc;
//        tmpVel.x = tmpVel.y = tmpVel.z = 0;
//        cmdMsg.velocity = tmpVel;
//        tmpPos.x = end_pt_(0);
//        tmpPos.y = end_pt_(1);
//        tmpPos.z = end_pt_(2);
//        cmdMsg.position = tmpPos;
//
//        // yaw
//        cmdMsg.yaw = yaw_angle_;
//
//        // draw cmd
//        Eigen::Vector3d dir(cos(yaw_angle_), sin(yaw_angle_), 0.0);
//        drawCmd(end_pt_, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
//
//        cmdMsg.header.stamp = ros::Time::now();
//        pos_cmd_pub_.publish(cmdMsg);
//    }
//}

void AdaptiveReplanFsm::highMpccCallback(const ros::TimerEvent& e){
    if (!have_low_traj_ || !have_odom_) return;
    double dist = (odom_pos_ - end_pt_).norm();
    if (dist < 0.5){
        cout << "near the goal " << endl;
        planner_manager_->resetMPCCinitial();
        have_low_traj_ = false;
        have_traj_     = false;
        have_target_   = false;
        near_goal_     = true;
        return;
    }

    // get the start state
    ros::Time T = ros::Time::now();
    Eigen::Vector3d start_state;
    bool success;
    if (have_traj_){
        tMpc2_ = ros::Time::now();
        double t = (tMpc2_ - tMpc1_).toSec() + mpcc_opt_;
        int k = floor(t/mpc_delta_T_);
        t = t - k * mpc_delta_T_;
        Eigen::Vector3d pos_L, vel_L;
        vel_L = planner_manager_->high_mpcc_traj_vel_[k];
        pos_L = planner_manager_->high_mpcc_traj_pos_[k];
//        start_state(0) = pos_L(0) + vel_L(0) * cos(pos_L(2)) * t;
//        start_state(1) = pos_L(1) + vel_L(0) * sin(pos_L(2)) * t;
//        start_state(2) = pos_L(2) + vel_L(1) * t;
        start_state(0) = odom_pos_(0);
        start_state(1) = odom_pos_(1);
        start_state(2) = yaw_angle_;

        // record the last mpcc
        tMpc1_lst_ = tMpc1_;
        tMpc1_ = tMpc2_ + ros::Duration(mpcc_opt_);
        high_mpcc_traj_pos_ = planner_manager_->high_mpcc_traj_pos_;
        high_mpcc_traj_vel_ = planner_manager_->high_mpcc_traj_vel_;
        success = callHighMpcc(start_state);
    } else{
        start_state(0) = odom_pos_(0);
        start_state(1) = odom_pos_(1);
        start_state(2) = yaw_angle_;
        success = callHighMpcc(start_state);
        tMpc1_ = ros::Time::now();
    }
    double t = (ros::Time::now() - T).toSec();
    if (t > T_max_highmpcc_){
        T_max_highmpcc_ = t;
    }
    if (show_highmpcc_time_){
        cout << "High mpcc:  cur t = " << t << " , max t = " <<  T_max_highmpcc_ << endl;
    }

    if(success){
        have_traj_   = true;
    } else {
        return;
    }
}

void AdaptiveReplanFsm::lowMpcCallback(const ros::TimerEvent& e){
    if (!have_odom_)    return;
    if (!have_target_)  return;
    ros::Time T = ros::Time::now();
    bool success = callLowMpc();
    // print the low mpc time
    double t = (ros::Time::now() - T).toSec();
    if (t > T_max_lowmpc_) T_max_lowmpc_ = t;
    if (show_lowmpc_time_)
        cout << "Low mpc:  cur t = " << t << " , max t = " <<  T_max_lowmpc_ << endl;

    if(success){
        have_low_traj_   = true;
    } else {
        return;
    }
}

void AdaptiveReplanFsm::trajSafetyCallback(const ros::TimerEvent& e){
    if (!have_low_traj_) return;
    if (!have_target_)  return;

//    std::cout << "Safety check" << std::endl;
    if (planner_manager_->safeCheck()){
        return; // safe
    } else{
        cout << "low traj is in obs, replan right now!" << endl;
        callLowMpc();
    }
}

bool AdaptiveReplanFsm::callHighMpcc(const Eigen::Vector3d& start_state){
    bool plan_path_success = planner_manager_->HighMpcc(start_state, if_adaptive_);
    if (plan_path_success){
        publishPathMessage(planner_manager_->high_mpcc_traj_pos_);
        return true;
    } else {
        cout << "generate new high mpcc fail." << endl;
        return false;
    }
}

bool AdaptiveReplanFsm::callLowMpc(){
    // Low mpc to optimize A* trajectory
    start_pt_  = odom_pos_;
    bool plan_path_success = planner_manager_->LowMpc(start_pt_,end_pt_);
    if (plan_path_success){
        have_low_traj_ = true;
//        publishPathMessage(planner_manager_->low_mpc_traj_);
        return true;
    }
    return false;
}

void AdaptiveReplanFsm::publishPathMessage(const std::vector<Eigen::Vector2d>& path) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  for (const auto &loc2d : path) {
    path_msg.poses.push_back(getRos3DPoseFromAstar2DLocation(loc2d[0], loc2d[1]));
  }
  path_pub_.publish(path_msg);
}

void AdaptiveReplanFsm::publishPathMessage(const std::vector<Eigen::Vector3d>& path) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  for (const auto &loc2d : path) {
    path_msg.poses.push_back(getRos3DPoseFromAstar2DLocation(loc2d[0], loc2d[1]));
  }
  path_pub_.publish(path_msg);
}

geometry_msgs::PoseStamped AdaptiveReplanFsm::getRos3DPoseFromAstar2DLocation(double x, double y) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  return pose;
}

}	// namespace adaptive_planner
