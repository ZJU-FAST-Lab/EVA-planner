#ifndef _LOW_MPC_H
#define _LOW_MPC_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include "plan_env/edt_environment.h"
#include <math.h>

#define inf 1000000.0
namespace adaptive_planner{

class low_mpc_planner
{
private:
    /* record data */
    EDTEnvironment::Ptr edt_map_;
    Eigen::Vector3d start_pt_, end_pt_;
    Eigen::VectorXd input_x_, input_y_, input_z_;  // input (v)
    Eigen::VectorXd state_x_, state_y_, state_z_;  // state of low mpc system
    Eigen::VectorXd path_x_, path_y_, path_z_;     // reference path 
    Eigen::Vector3d map_min_, map_max_;
    double f_;
    
    /* low mpc setting */
    int N_;                 // prediction horizon number
    double Ts_;             // time step
    double dist_0_;         // obstacle distance threshold
    Eigen::MatrixXd A_;     // state equation (S = A_ * U + B_ * S0)
    Eigen::VectorXd B_;

    /* optimization parameters */
    double alpha1_;     // control input weight
    double alpha2_;     // distance to obstacle weight
    double alpha3_;     // similarity to reference path
    Eigen::VectorXd Gradient_x_, Gradient_y_, Gradient_z_;

    int max_iteration_num_, iter_num_;
    double similary_lower_, similary_upper_;
    double max_iteration_time_, min_cost_;
    int dim_, variable_num_;
    std::vector<double> best_variable_;
    
    /* useful function */
    void setInitial(Eigen::Vector3d start_pt,std::vector<Eigen::Vector3d> local_path);
    void optimize();
    static double costFunction(const std::vector<double>& x, 
                               std::vector<double>& grad,
                               void* func_data);
    void combineCost(const std::vector<double>& x,
                     std::vector<double>& grad,  
                     double& f_combine);
    void stateEquations();                             // use the state equations to calculate system states
    void calCostFunctionandGradient();                 // calculate the cost and gradient
    double calPathRangeCost(double value);
    double calPathRangeGrad(double value);

public:
    low_mpc_planner(){};
    ~low_mpc_planner(){};

    /* main API */
    void setEnvironment(const EDTEnvironment::Ptr& env);
    void init(ros::NodeHandle& nh);
    std::vector<Eigen::Vector3d>  lowMpcOptimization(Eigen::Vector3d start_pt, std::vector<Eigen::Vector3d> local_path);

    typedef unique_ptr<low_mpc_planner> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}    // namespace adaptive_planner

#endif