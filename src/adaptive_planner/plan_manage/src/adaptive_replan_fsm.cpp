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
    
    nh.param("visual/line_width",               line_width_, -1.0);
    nh.param("multi_mpcc/Ts",                   mpc_delta_T_, -1.0);
    nh.param("multi_mpcc/Mpcc_opt",             mpcc_opt_, -1.0);
    nh.param("optimization/vel_upper",         v_max_,-1.0);
    nh.param("multi_mpcc/show_lowmpc_time",     show_lowmpc_time_, false);
    nh.param("multi_mpcc/show_highmpcc_time",   show_highmpcc_time_, false);
    nh.param("fsm/flight_type",                 flight_type_, 0);
    v_min_ = 0.0;
    // get the goal waypoints
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
    if (waypoint_num_>=1)
        waypoint_flag_ = 0;     // if there are some waypoints, then initial the first goal
    
    yaw_0_ << 1.0,0.0;
    /* initialize main modules */
    planner_manager_.reset(new AdaptivePlannerManager);
    planner_manager_->initPlanModules(nh);
    visualization_.reset(new PlanningVisualization(nh));
    /* callback */
    cmd_timer_          = nh.createTimer(ros::Duration(0.01), &AdaptiveReplanFsm::cmdCallback, this);
    low_mpc_timer_      = nh.createTimer(ros::Duration(2.0), &AdaptiveReplanFsm::lowMpcCallback, this);
    high_mpcc_timer_    = nh.createTimer(ros::Duration(mpc_delta_T_), &AdaptiveReplanFsm::highMpccCallback, this);
    safety_check_timer_ = nh.createTimer(ros::Duration(0.05), &AdaptiveReplanFsm::trajSafetyCallback, this);

    odom_sub_           = nh.subscribe("/odom_world", 1, &AdaptiveReplanFsm::odometryCallback, this);
    // joy_sub_      = nh.subscribe("/joy", 1,  &AdaptiveReplanFsm::joyCallback,this);
    pos_cmd_pub_        = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);   
    pos_cmd_vis_pub_    = nh.advertise<visualization_msgs::Marker> ("/position_cmd_vis", 50);
    exected_traj_pub_   = nh.advertise<visualization_msgs::Marker> ("/exected_traj", 50);
    
    if_adaptive_  = true;
    T_max_lowmpc_ = T_max_highmpcc_ = 0.0;

    if (flight_type_ == TARGET_TYPE::MANUAL_TARGET){
        waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &AdaptiveReplanFsm::waypointCallback, this);
    } else if (flight_type_ == TARGET_TYPE::PRESET_TARGET){
        trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &AdaptiveReplanFsm::triggerCallback, this);
        ros::Duration(1.0).sleep();
        ROS_WARN("Waiting for trigger from [n3ctrl] from RC");
        while (ros::ok() && (!have_odom_ || !have_trigger_))
        {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }
    } else
      cout << "Wrong target_type_ value! target_type_=" << flight_type_ << endl;
}

void AdaptiveReplanFsm::waypointCallback(const nav_msgs::PathConstPtr& msg){
    if (msg->poses[0].pose.position.z < -0.1) return;

    cout << "Get waypoint!" << endl;
    // manual target type
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    visualization_->drawLocalGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    planner_manager_->resetMPCCinitial();

    // waypoints target type
    // if (waypoint_flag_ == 0){
    //     end_pt_ << waypoints_[0][0], waypoints_[0][1], waypoints_[0][2];
    //     visualization_->drawLocalGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    //     planner_manager_->resetMPCCinitial();
    // }

    have_target_ = true;
    near_goal_   = false;
}

void AdaptiveReplanFsm::triggerCallback(const geometry_msgs::PoseStampedPtr &msg){
    have_trigger_ = true;
    cout << "Triggered!" << endl;

    if (waypoint_flag_ == 0){
        end_pt_ << waypoints_[0][0], waypoints_[0][1], waypoints_[0][2];
        visualization_->drawLocalGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        planner_manager_->resetMPCCinitial();
    }

    have_target_ = true;
    near_goal_   = false;
}

void AdaptiveReplanFsm::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // if need to change the goal
    // if ((end_pt_-odom_pos_).norm() <= 1.5 && waypoint_flag_!=waypoint_num_-1 && have_trigger_){
    // if ((end_pt_-odom_pos_).norm() <= 1.5 && waypoint_flag_!=waypoint_num_-1){
    //     // near a temp goal and need to change to next one
    //     waypoint_flag_ ++;
    //     cout << "change the goal num: " << waypoint_flag_ << endl;
    //     end_pt_ << waypoints_[waypoint_flag_][0], waypoints_[waypoint_flag_][1], waypoints_[waypoint_flag_][2];
    //     visualization_->drawLocalGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    //     callLowMpc();
    // }

    // record history pos and vel
    if (have_target_){
        history_pos_.push_back(odom_pos_);
        history_vel_.push_back(odom_vel_);
    }

    have_odom_ = true;
}

// void AdaptiveReplanFsm::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
//     if_adaptive_ =  msg->buttons[0];
//     if (if_adaptive_){
//         cout << " [Adaptive mode] -----------" << endl;
//     } else {
//         cout << " [Normal mode] -------------" << endl;
//     }
// }

void AdaptiveReplanFsm::cmdCallback(const ros::TimerEvent& e){
    if (have_traj_)
    {
        geometry_msgs::Point tmpPos;
        geometry_msgs::Vector3 tmpVel, tmpAcc;
        quadrotor_msgs::PositionCommand cmdMsg;
        cmdMsg.header.frame_id = "world";
        cmdMsg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        double t;
        int k;
        Eigen::Vector3d pos_L, pos_R, vel_L, vel_R, acc_L, acc_R,pos_yaw;
        if (high_mpcc_traj_pos_.size() == 0){
            t = (ros::Time::now() - tMpc1_).toSec();
            k = floor(t/mpc_delta_T_);
            // cout <<"[0]  k:  "<< k << "  t:  " << t << endl;
            acc_L = planner_manager_->high_mpcc_traj_acc_[k];
            acc_R = planner_manager_->high_mpcc_traj_acc_[k+1];
            vel_L = planner_manager_->high_mpcc_traj_vel_[k];
            vel_R = planner_manager_->high_mpcc_traj_vel_[k+1];
            pos_L = planner_manager_->high_mpcc_traj_pos_[k];
            pos_R = planner_manager_->high_mpcc_traj_pos_[k+1];
        }else if ((ros::Time::now()-tMpc2_).toSec() - mpcc_opt_ < 0  ){
            t = (ros::Time::now() - tMpc1_lst_).toSec();
            k = floor(t/mpc_delta_T_);
            // cout <<"[1]  k: " << k << "  t:  " << t << endl;
            acc_L = high_mpcc_traj_acc_[k];
            acc_R = high_mpcc_traj_acc_[k+1];
            vel_L = high_mpcc_traj_vel_[k];
            vel_R = high_mpcc_traj_vel_[k+1];
            pos_L = high_mpcc_traj_pos_[k];
            pos_R = high_mpcc_traj_pos_[k+1];
        }else {
            t = (ros::Time::now() - tMpc1_).toSec();
            k = floor(t/mpc_delta_T_);
            // cout <<"[2]  k: " << k << "  t:  " << t << endl;
            acc_L = planner_manager_->high_mpcc_traj_acc_[k];
            acc_R = planner_manager_->high_mpcc_traj_acc_[k+1];
            vel_L = planner_manager_->high_mpcc_traj_vel_[k];
            vel_R = planner_manager_->high_mpcc_traj_vel_[k+1];
            pos_L = planner_manager_->high_mpcc_traj_pos_[k];
            pos_R = planner_manager_->high_mpcc_traj_pos_[k+1];
        }
        t = t - k * mpc_delta_T_;
        tmpAcc.x = acc_L(0) + (acc_R(0) - acc_L(0)) * t / mpc_delta_T_;
        tmpAcc.y = acc_L(1) + (acc_R(1) - acc_L(1)) * t / mpc_delta_T_;
        tmpAcc.z = acc_L(2) + (acc_R(2) - acc_L(2)) * t / mpc_delta_T_;
        cmdMsg.acceleration = tmpAcc;
        tmpVel.x = vel_L(0) + acc_L(0) * t + (acc_R(0) - acc_L(0)) * t * t / mpc_delta_T_ / 2;
        tmpVel.y = vel_L(1) + acc_L(1) * t + (acc_R(1) - acc_L(1)) * t * t / mpc_delta_T_ / 2;
        tmpVel.z = vel_L(2) + acc_L(2) * t + (acc_R(2) - acc_L(2)) * t * t / mpc_delta_T_ / 2;
        cmdMsg.velocity = tmpVel;
        tmpPos.x = pos_L(0) + vel_L(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
        tmpPos.y = pos_L(1) + vel_L(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
        tmpPos.z = pos_L(2) + vel_L(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;
        cmdMsg.position = tmpPos;
        
        // yaw 
        Eigen::Vector2d temp_vec;
        temp_vec << tmpVel.x, tmpVel.y;
        double yaw_angle_cos = yaw_0_.dot(temp_vec) / yaw_0_.norm() / temp_vec.norm();
        double yaw_angle;
        if (temp_vec(1) < 0) {
            yaw_angle = -acos(yaw_angle_cos);
        } else{
            yaw_angle = acos(yaw_angle_cos);
        }
        yaw_angle_ = yaw_angle;
        Eigen::Vector2d temp_acc;
        temp_acc << tmpAcc.x, tmpAcc.y;
        double yaw_dot;
        if (tmpVel.x * tmpAcc.y >= tmpVel.y * tmpAcc.x){
            yaw_dot = temp_acc.norm();
        } else{
            yaw_dot = -temp_acc.norm();
        }
        cmdMsg.yaw = yaw_angle;
        cmdMsg.yaw_dot = yaw_dot;

        // draw cmd
        Eigen::Vector3d pos,vel,acc;
        pos << tmpPos.x, tmpPos.y, tmpPos.z;
        vel << tmpVel.x, tmpVel.y, tmpVel.z;
        acc << tmpAcc.x, tmpAcc.y, tmpAcc.z;
        drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
        drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
        Eigen::Vector3d dir(cos(yaw_angle), sin(yaw_angle), 0.0);
        drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

        // draw exected_traj with vel color
        // ADD

        cmdMsg.header.stamp = ros::Time::now();
        pos_cmd_pub_.publish(cmdMsg);
    }
    if (near_goal_){
        geometry_msgs::Point tmpPos;
        geometry_msgs::Vector3 tmpVel, tmpAcc;
        quadrotor_msgs::PositionCommand cmdMsg;
        cmdMsg.header.frame_id = "world";
        cmdMsg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        tmpAcc.x = tmpAcc.y = tmpAcc.z = 0;
        cmdMsg.acceleration = tmpAcc;
        tmpVel.x = tmpVel.y = tmpVel.z = 0;
        cmdMsg.velocity = tmpVel;
        tmpPos.x = end_pt_(0);
        tmpPos.y = end_pt_(1);
        tmpPos.z = end_pt_(2);
        cmdMsg.position = tmpPos;

        // yaw
        cmdMsg.yaw = yaw_angle_;

        // draw cmd
        Eigen::Vector3d dir(cos(yaw_angle_), sin(yaw_angle_), 0.0);
        drawCmd(end_pt_, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

        cmdMsg.header.stamp = ros::Time::now();
        pos_cmd_pub_.publish(cmdMsg);
    }
}

void AdaptiveReplanFsm::highMpccCallback(const ros::TimerEvent& e){
    if (!have_low_traj_) return;
    double dist = (odom_pos_ - end_pt_).norm();
    if (dist < 0.5){
        cout << "near the goal " << endl;
        planner_manager_->resetMPCCinitial();
        draw_history_traj();
        have_low_traj_ = false;
        have_traj_     = false;
        have_target_   = false;
        near_goal_     = true;
        return;
    }
    
    // get the start state 
    ros::Time T = ros::Time::now();
    Eigen::Matrix3d start_state;
    bool success;
    if (have_traj_){
        tMpc2_ = ros::Time::now();
        double t = (tMpc2_ - tMpc1_).toSec() + mpcc_opt_;
        int k = floor(t/mpc_delta_T_);
        t = t - k * mpc_delta_T_;
        Eigen::Vector3d pos_L, pos_R, vel_L, vel_R, acc_L, acc_R;
        acc_L = planner_manager_->high_mpcc_traj_acc_[k];
        acc_R = planner_manager_->high_mpcc_traj_acc_[k+1];
        vel_L = planner_manager_->high_mpcc_traj_vel_[k];
        vel_R = planner_manager_->high_mpcc_traj_vel_[k+1];
        pos_L = planner_manager_->high_mpcc_traj_pos_[k];
        pos_R = planner_manager_->high_mpcc_traj_pos_[k+1];
        start_state(2,0) = acc_L(0) + (acc_R(0) - acc_L(0)) * t / mpc_delta_T_;
        start_state(2,1) = acc_L(1) + (acc_R(1) - acc_L(1)) * t / mpc_delta_T_;
        start_state(2,2) = acc_L(2) + (acc_R(2) - acc_L(2)) * t / mpc_delta_T_;
        start_state(1,0) = vel_L(0) + acc_L(0) * t + (acc_R(0) - acc_L(0)) * t * t / mpc_delta_T_ / 2;
        start_state(1,1) = vel_L(1) + acc_L(1) * t + (acc_R(1) - acc_L(1)) * t * t / mpc_delta_T_ / 2;
        start_state(1,2) = vel_L(2) + acc_L(2) * t + (acc_R(2) - acc_L(2)) * t * t / mpc_delta_T_ / 2;
        start_state(0,0) = pos_L(0) + vel_L(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
        start_state(0,1) = pos_L(1) + vel_L(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
        start_state(0,2) = pos_L(2) + vel_L(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;
        
        // start_state(1,0) = odom_vel_(0) + acc_L(0) * t + (acc_R(0) - acc_L(0)) * t * t / mpc_delta_T_ / 2;
        // start_state(1,1) = odom_vel_(1) + acc_L(1) * t + (acc_R(1) - acc_L(1)) * t * t / mpc_delta_T_ / 2;
        // start_state(1,2) = odom_vel_(2) + acc_L(2) * t + (acc_R(2) - acc_L(2)) * t * t / mpc_delta_T_ / 2;
        // start_state(0,0) = odom_pos_(0) + odom_vel_(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
        // start_state(0,1) = odom_pos_(1) + odom_vel_(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
        // start_state(0,2) = odom_pos_(2) + odom_vel_(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;

        // record the last mpcc 
        tMpc1_lst_ = tMpc1_;
        tMpc1_ = tMpc2_ + ros::Duration(mpcc_opt_);
        high_mpcc_traj_pos_ = planner_manager_->high_mpcc_traj_pos_;
        high_mpcc_traj_vel_ = planner_manager_->high_mpcc_traj_vel_;
        high_mpcc_traj_acc_ = planner_manager_->high_mpcc_traj_acc_;
        success = callHighMpcc(start_state);
    } else{
        start_state(0,0) = odom_pos_(0);
        start_state(0,1) = odom_pos_(1);
        start_state(0,2) = odom_pos_(2);
        start_state(1,0) = odom_vel_(0);
        start_state(1,1) = odom_vel_(1);
        start_state(1,2) = odom_vel_(2);
        start_state(2,0) = 0.0;
        start_state(2,1) = 0.0;
        start_state(2,2) = 0.0;
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
        draw_history_traj();
        have_traj_   = true;
        // have_low_traj_ = false; // debug , wait to be deleted
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
        // have_target_     = false;   // debug 
    } else {
        return;
    }
}

void AdaptiveReplanFsm::trajSafetyCallback(const ros::TimerEvent& e){
    if (!have_low_traj_) return;
    if (!have_target_)  return;

    bool if_safe = planner_manager_->safeCheck();
    if (if_safe){
        return; // safe
    } else{
        cout << "low traj is in obs, replan right now!" << endl;
        callLowMpc();
    }
}

bool AdaptiveReplanFsm::callHighMpcc(Eigen::Matrix3d start_state){
    bool plan_path_success = planner_manager_->HighMpcc(start_state, if_adaptive_);
    if (plan_path_success){

        visualization_->drawHighMpccTraj(planner_manager_->high_mpcc_traj_pos_, 0.2,Eigen::Vector4d(1, 0, 0, 1.0));
        visualization_->drawHighMpccRefTraj(planner_manager_->high_mpcc_traj_ref_, 0.1,Eigen::Vector4d(0, 0, 1, 1.0));
        return true;
    } else {
        cout << "generate new high mpcc fail." << endl;
        return false;
    }
}

bool AdaptiveReplanFsm::callLowMpc(){
    // calculate the start pos
    // if (have_traj_){
    //     ros::Time temp_t = ros::Time::now();
    //     double mpc_opt = 0.003;
    //     double t = (temp_t - tMpc1_).toSec() + mpc_opt;
    //     int k = floor(t/mpc_delta_T_);
    //     t = t - k * mpc_delta_T_;
    //     Eigen::Vector3d pos_L, pos_R, vel_L, vel_R, acc_L, acc_R;
    //     acc_L = planner_manager_->high_mpcc_traj_acc_[k];
    //     acc_R = planner_manager_->high_mpcc_traj_acc_[k+1];
    //     vel_L = planner_manager_->high_mpcc_traj_vel_[k];
    //     vel_R = planner_manager_->high_mpcc_traj_vel_[k+1];
    //     pos_L = planner_manager_->high_mpcc_traj_pos_[k];
    //     pos_R = planner_manager_->high_mpcc_traj_pos_[k+1];
    //     start_pt_(0) = pos_L(0) + vel_L(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
    //     start_pt_(1) = pos_L(1) + vel_L(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
    //     start_pt_(2) = pos_L(2) + vel_L(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;
    // } else{
    //     start_pt_  = odom_pos_;
    // }
    start_pt_  = odom_pos_;
    bool plan_path_success = planner_manager_->LowMpc(start_pt_,end_pt_);
    if (plan_path_success){
        /* visulization */
        visualization_->drawPath(planner_manager_->local_path_, line_width_, Eigen::Vector4d(0, 0, 1, 1.0));
        visualization_->drawLowMpcTraj(planner_manager_->low_mpc_traj_,0.1, Eigen::Vector4d(0, 1, 0, 1.0));
        have_low_traj_ = true;
        return true;
    }
    return false;
}

void AdaptiveReplanFsm::drawCmd(const Eigen::Vector3d& pos, 
                                const Eigen::Vector3d& vec, 
                                const int& id,
                                const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  pos_cmd_vis_pub_.publish(mk_state);
}

void AdaptiveReplanFsm::draw_history_traj(){
    unsigned int sp = history_pos_.size(), sv = history_vel_.size();
    int size = sp<=sv ? sp : sv;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::LINE_STRIP;
    // mk.action          = visualization_msgs::Marker::DELETE;

    mk.action             = visualization_msgs::Marker::ADD;
    mk.scale.x = 0.15;
    for (int i = 0; i < size; i++){
        Eigen::Vector3d v = history_vel_[i];
        double v_norm = v.norm(); 
        // cout << "v  :  " << v_norm << endl;
        double color_num;
        if(v_norm < v_min_)
            color_num = 0.0;
        else if(v_norm > v_max_)
            color_num = 1.0;
        else{
            color_num = (v_norm - v_min_) / (v_max_ - v_min_);
        } 

        std_msgs::ColorRGBA color;    
        color.r = color_num>0.5 ? 2*color_num-1 : 0.0;
        color.g = color_num>0.5 ? -2*color_num+2 : 2*color_num;
        color.b = color_num>0.5 ? 0.0 : -2*color_num+1;
        color.a = 1.0;
        mk.colors.push_back(color);

        geometry_msgs::Point pt;
        pt.x = history_pos_[i](0);
        pt.y = history_pos_[i](1);
        pt.z = history_pos_[i](2);
        mk.points.push_back(pt);
    }
    exected_traj_pub_.publish(mk);
    // ros::Duration(0.001).sleep();
}

}	// namespace adaptive_planner