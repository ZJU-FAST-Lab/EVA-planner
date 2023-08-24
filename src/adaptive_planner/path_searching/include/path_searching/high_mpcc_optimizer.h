#ifndef _HIGH_MPCC_OPTIMIZER_H
#define _HIGH_MPCC_OPTIMIZER_H

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include "plan_env/edt_environment.h"
#include <math.h>

#define inf 1000000.0
namespace adaptive_planner{

class high_mpcc_optimizer
{
private:
    /* record data */
    EDTEnvironment::Ptr edt_map_;
    Eigen::Vector3d start_state_;
    double mu_start_state_;
    std::vector<Eigen::Vector2d> low_mpc_traj_;
    int low_traj_num_;
    double low_traj_T_;
    double f_;
    Eigen::VectorXd input_u_, input_w_, input_mu_, input_mux_x_, input_mux_y_;  // input (linear and angular velocity)
    Eigen::VectorXd state_x_, state_y_, state_theta_, state_mu_;  // state of high mpcc system and reference theta system
    Eigen::VectorXd theta_mux_input_;
    Eigen::Vector2d map_min_, map_max_;
    Eigen::VectorXd Gradient_u_, Gradient_w_, Gradient_mu_;

    /* high mpcc setting */
    int N_;                 // prediction horizon number
    double Ts_;             // time step
    double dist_0_, dist_1_;         // obstacle distance threshold 
    double K_;              // penalty coefficient
    double vel_min_;        
    Eigen::MatrixXd A_, A_mux_x_, A_mux_y_;     // state equation (S = A_ * U + B_ * S0)
    Eigen::MatrixXd A_mux_x_theta_, A_mux_y_theta_;
    Eigen::MatrixXd B_;

    /* optimization parameters */
    double alpha_s_;                  // the cost of similarity
    double alpha_p_;                  // the cost of progress value
    double alpha_e_, alpha3_tmp_;     // velocity direction dependent penalty
    double alpha_c_;                  // avoid collisions
    double alpha_mu_;                  // the penalty of theta , which is used to prevent theta from exceeding the range
    double alpha6_;                  // the penalty of vel and acc, which is used to prevent vel and acc from exceeding the range
    double alpha7_;                  // minimum input jerk
    double alpha8_;
    double vel_u_lower_, vel_u_upper_, vel_w_lower_, vel_w_upper_, mu_lower_, mu_upper_;
    int max_iteration_num_, iter_num_;
    double max_iteration_time_;
    double min_cost_;
    std::vector<double> best_variable_;
    int dim_, variable_num_;

    /* useful function */
    void setInitialState( Eigen::Vector3d& start_state,
                          std::vector<Eigen::Vector2d>& low_mpc_traj);
    void setInitialSystem();
    Eigen::Vector2d muGetPosition(double t);
    void optimize();

    /* cost function */
    /* calculate each part of cost function with control points q as input */
    static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
    void combineCost(const std::vector<double>& x, std::vector<double>& grad,  double& f_combine);
    void stateEquations();
    void calCostFunctionandGradient();
    double calRangeCost(double value, double lower,double upper);
    double calRangeGrad(double value, double lower,double upper);
    double calvmuRangeCost(double value);
    double calvmuRangeGrad(double value);

public:
    high_mpcc_optimizer(){};
    ~high_mpcc_optimizer(){};

    /* main API */
    void resetInputInital();
    void setEnvironment(const EDTEnvironment::Ptr& env);
    void setParam(ros::NodeHandle& nh);
    std::vector<std::vector<Eigen::Vector3d>> mpccOptimizeTraj(Eigen::Vector3d start_state,
                                                               std::vector<Eigen::Vector2d> low_mpc_traj,
                                                               bool if_adaptive);
                                                                   
    typedef unique_ptr<high_mpcc_optimizer> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}    // namespace adaptive_planner

#endif
