#ifndef _HIGH_MPCC_OPTIMIZER_H
#define _HIGH_MPCC_OPTIMIZER_H

#include <Eigen/Eigen>
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
    Eigen::Matrix3d start_state_;
    Eigen::Vector3d theta_start_state_;
    std::vector<Eigen::Vector3d> low_mpc_traj_;
    int low_traj_num_;
    double low_traj_T_;
    double f_;
    Eigen::VectorXd input_x_, input_y_, input_z_, input_theta_;  // input (jerk)
    Eigen::VectorXd state_x_, state_y_, state_z_, state_theta_;  // state of high mpcc system and reference theta system
    Eigen::Vector3d map_min_, map_max_;
    Eigen::VectorXd Gradient_x_, Gradient_y_, Gradient_z_, Gradient_theta_;    

    /* high mpcc setting */
    int N_;                 // prediction horizon number
    double Ts_;             // time step
    double dist_0_, dist_1_;         // obstacle distance threshold 
    double K_;              // penalty coefficient
    double vel_min_;        
    Eigen::MatrixXd A_;     // state equation (S = A_ * U + B_ * S0)
    Eigen::MatrixXd B_;

    /* optimization parameters */
    double alpha1_;                  // the cost of similarity 
    double alpha2_;                  // the cost of progress value
    double alpha3_, alpha3_tmp_;     // velocity directioin depend panelty
    double alpha4_;                  // avoid collisions
    double alpha5_;                  // the penalty of theta , which is used to prevent theta from exceeding the range 
    double alpha6_;                  // the penalty of vel and acc, which is used to prevent vel and acc from exceeding the range
    double alpha7_;                  // minimum input jerk
    double alpha8_;
    double vel_lower_, vel_upper_, acc_lower_, acc_upper_, theta_lower_, theta_upper_, jerk_lower_, jerk_upper_;
    int max_iteration_num_, iter_num_;
    double max_iteration_time_;
    double min_cost_;
    std::vector<double> best_variable_;
    int dim_, variable_num_;

    /* useful function */
    void setInitialState( Eigen::Matrix3d start_state,
                          std::vector<Eigen::Vector3d> low_mpc_traj);
    void setInitialSystem();
    Eigen::Vector3d thetaGetPosition(double t);
    Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
    void optimize();

    /* cost function */
    /* calculate each part of cost function with control points q as input */
    static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
    void combineCost(const std::vector<double>& x, std::vector<double>& grad,  double& f_combine);
    void stateEquations();
    void calCostFunctionandGradient();
    double calRangeCost(double value, double lower,double upper);
    double calRangeGrad(double value, double lower,double upper);
    double calvthetaRangeCost(double value);
    double calvthetaRangeGrad(double value);

public:
    high_mpcc_optimizer(){};
    ~high_mpcc_optimizer(){};

    /* main API */
    void resetInputInital();
    void setEnvironment(const EDTEnvironment::Ptr& env);
    void setParam(ros::NodeHandle& nh);
    std::vector<std::vector<Eigen::Vector3d>> mpccOptimizeTraj(Eigen::Matrix3d start_state,
                                                               std::vector<Eigen::Vector3d> low_mpc_traj,
                                                               bool if_adaptive);
                                                                   
    typedef unique_ptr<high_mpcc_optimizer> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}    // namespace adaptive_planner

#endif