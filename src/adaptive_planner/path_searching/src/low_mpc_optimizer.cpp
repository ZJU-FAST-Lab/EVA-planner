#include <path_searching/low_mpc.h>
#include <nlopt.hpp>

using namespace std;
using namespace Eigen;

namespace adaptive_planner{

void low_mpc_planner::init(ros::NodeHandle& nh){
    nh.param("multi_mpcc/Ts", Ts_, -1.0);
    nh.param("low_mpc/dist_0", dist_0_, -1.0);

    nh.param("low_mpc/alpha1", alpha1_, -1.0);
    nh.param("low_mpc/alpha2", alpha2_, -1.0);
    nh.param("low_mpc/alpha3", alpha3_, -1.0);

    nh.param("low_mpc/max_iteration_num", max_iteration_num_, -1);
    nh.param("low_mpc/max_iteration_time", max_iteration_time_, -1.0);
    nh.param("low_mpc/similary_lower", similary_lower_, -1.0);
    nh.param("low_mpc/similary_upper", similary_upper_, -1.0);
}

void low_mpc_planner::setEnvironment(const EDTEnvironment::Ptr& env) {
    this->edt_map_ = env;
}

vector<Vector3d> low_mpc_planner::lowMpcOptimization(Vector3d start_pt, vector<Vector3d> local_path){
    // ros::Time tMpc = ros::Time::now();
    
    // set initial system
    setInitial(start_pt, local_path);

    // use the NLopt optimizer
    optimize();

    // return the optimized path
    vector<Vector3d> traj;
    for (int i = 0; i < N_; i++){
        Vector3d pos;
        pos << state_x_(i), state_y_(i), state_z_(i) ;
        traj.push_back(pos);
    }

    // debug 
    /*double dist;
    Vector3d dist_grad;
    for (int i = 0; i < N_; i++){
        Vector3d pos;
        pos << state_x_(i), state_y_(i), state_z_(i) ;
        edt_map_->evaluateEDT(pos, dist);
        edt_map_->evaluateFirstGrad(pos, dist_grad);
        cout << "node num:  " << i << "-------------------" << endl;
        cout << "dist     :    " << dist << endl;
        cout << "dist grad:    " << dist_grad.transpose() << endl; 
    }  
    cout << "------------------- " << endl;
    cout << "iter_num_    : " << iter_num_ << endl; 
    double t_opt = (ros::Time::now() - tMpc).toSec();
    cout << "time         : " << t_opt << endl;*/

    return traj;
}

void low_mpc_planner::stateEquations(){
    state_x_ = A_ * input_x_ + B_ * start_pt_(0);
    state_y_ = A_ * input_y_ + B_ * start_pt_(1);
    state_z_ = A_ * input_z_ + B_ * start_pt_(2);
}

void low_mpc_planner::setInitial(Vector3d start_pt,vector<Vector3d> local_path){
    // initial start and end state
    start_pt_ = start_pt;
    end_pt_   = local_path.back();
    N_        = local_path.size();
    dim_      = 3;                    // x, y, z
    // get the state matrix of the low mpc
    A_ = MatrixXd::Zero(N_,N_-1);
    B_ = VectorXd::Zero(N_);
    B_(0) = 1;
    for (int i = 1 ; i < N_ ; i++){
        for (int j = 0;j < i ; j++){
            A_(i,j) = Ts_;
        }
        B_(i) = 1;
    }
    // initial state value and input
    state_x_    = VectorXd::Zero(N_);
    state_y_    = VectorXd::Zero(N_);
    state_z_    = VectorXd::Zero(N_);
    for (int i = 0; i < N_; i++){
        Vector3d pos = local_path[i];
        state_x_(i) = pos(0);
        state_y_(i) = pos(1);
        state_z_(i) = pos(2);
    }
    input_x_  = VectorXd::Zero(N_-1);
    input_y_  = VectorXd::Zero(N_-1);
    input_z_  = VectorXd::Zero(N_-1);
    for (int i = 0; i< N_-1; i++){
        input_x_(i) = (state_x_(i+1) - state_x_(i)) / Ts_;
        input_y_(i) = (state_y_(i+1) - state_y_(i)) / Ts_;
        input_x_(i) = (state_z_(i+1) - state_z_(i)) / Ts_;
    }

    // initial reference path
    path_x_ = state_x_;
    path_y_ = state_y_;
    path_z_ = state_z_;
}

void low_mpc_planner::optimize(){
    /* initialize solver */
    iter_num_      = 0;
    min_cost_      = std::numeric_limits<double>::max();
    variable_num_  = dim_ * (N_ -1);

    /* do optimization using NLopt slover */
    nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_); // use the LD_LBFGS optimization method
    opt.set_min_objective(low_mpc_planner::costFunction, this);
    opt.set_maxeval(max_iteration_num_);
    opt.set_maxtime(max_iteration_time_);
    opt.set_ftol_rel(1e-4);
    opt.set_vector_storage(16);

    // initial objective variables
    vector<double> q(variable_num_);
    for (int i = 0; i < (N_ - 1);i++){
        q[i] = input_x_(i);
    }
    for (int i = (N_ - 1); i < 2*(N_ - 1);i++){
        q[i] = input_y_(i-(N_ - 1));
    }
    for (int i = 2*(N_ - 1); i < 3*(N_ - 1);i++){
        q[i] = input_z_(i-2*(N_ - 1));
    }
    double final_cost;
    try
    {
        nlopt::result result = opt.optimize(q, final_cost);
    }
    catch(std::exception& e)
    {
        // ROS_WARN("[Optimization]: nlopt exception");
        // cout << e.what() << endl;
    }
}

double low_mpc_planner::costFunction(const std::vector<double>& x, 
                                     std::vector<double>& grad,
                                     void* func_data){
    low_mpc_planner* opt = reinterpret_cast<low_mpc_planner*>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);

    // debug 
    // cout << "iter_num_:   " << opt->iter_num_ << "  f:  " << cost << "-------------" << endl;
    
    opt->iter_num_++;
    /* save the min cost result */
    if (cost < opt->min_cost_) {
        opt->min_cost_      = cost;
        opt->best_variable_ = x;
    }
    return cost;
}

void low_mpc_planner::combineCost(const std::vector<double>& x,
                                  std::vector<double>& grad,  
                                  double& f_combine){
    /* convert the NLopt format vector to control inputs. */
    for (int i = 0; i < (N_ - 1);i++){
        input_x_(i) = x[i];
    }
    for (int i = (N_ - 1); i < 2*(N_ - 1);i++){
        input_y_(i-(N_ - 1)) = x[i];
    }
    for (int i = 2*(N_ - 1); i < 3*(N_ - 1);i++){
        input_z_(i-2*(N_ - 1)) = x[i];
    }
    /*  evaluate costs and their gradient  */
    f_combine = 0.0;
    grad.resize(variable_num_);
    fill(grad.begin(), grad.end(), 0.0);
    // calculate the initial system state
    stateEquations();
    // calculate the initial cost function and gradient
    calCostFunctionandGradient();
    f_combine = f_;

    // convert the gradient to the NLopt format
    for (int i = 0; i < (N_ - 1);i++){
        grad[i] = Gradient_x_(i);
    }
    for (int i = (N_ - 1); i < 2*(N_ - 1);i++){
        grad[i] = Gradient_y_(i-(N_ - 1));
    }
    for (int i = 2*(N_ - 1); i < 3*(N_ - 1);i++){
        grad[i] = Gradient_z_(i-2*(N_ - 1));
    }    
}

void low_mpc_planner::calCostFunctionandGradient(){
    // initial intermediate variables
    double fu, fc, fs, dist;
    fu = fc = fs = 0.0;
    Vector3d pos, path_pos, dist_grad;
    VectorXd Gradient_ux, Gradient_uy, Gradient_uz;
    VectorXd Gradient_cx, Gradient_cy, Gradient_cz;
    VectorXd Gradient_sx, Gradient_sy, Gradient_sz;
    Gradient_ux = Gradient_uy = Gradient_uz = VectorXd::Zero(N_-1);
    Gradient_cx = Gradient_cy = Gradient_cz = VectorXd::Zero(N_-1);
    Gradient_sx = Gradient_sy = Gradient_sz = VectorXd::Zero(N_-1);
    for (int i = 0; i<N_; i++){
        // fu and gradient of fu
        if (i < N_-1){
            if (i < N_ -2){
                fu += pow((input_x_(i+1) - input_x_(i)),2) 
                + pow((input_y_(i+1) - input_y_(i)),2) 
                + pow((input_z_(i+1) - input_z_(i)),2);
            }
            if (i == 0){
                Gradient_ux(i) = 2 * (input_x_(i) - input_x_(i+1));
                Gradient_uy(i) = 2 * (input_y_(i) - input_y_(i+1));
                Gradient_uz(i) = 2 * (input_z_(i) - input_z_(i+1));
            } else if ( i == N_-2){
                Gradient_ux(i) = 2 * (input_x_(i) - input_x_(i-1));
                Gradient_uy(i) = 2 * (input_y_(i) - input_y_(i-1));
                Gradient_uz(i) = 2 * (input_z_(i) - input_z_(i-1));
            } else {
                Gradient_ux(i) = 4 * input_x_(i) -  2 * (input_x_(i-1) + input_x_(i+1));
                Gradient_uy(i) = 4 * input_y_(i) -  2 * (input_y_(i-1) + input_y_(i+1));
                Gradient_uz(i) = 4 * input_z_(i) -  2 * (input_z_(i-1) + input_z_(i+1));
            }
        }

        //fc and gradient of fc
        if (i > 0){
            pos << state_x_(i),state_y_(i),state_z_(i);
            edt_map_->evaluateEDT(pos, dist);
            if (abs(dist) < dist_0_){
                fc += pow(dist - dist_0_, 2);
                edt_map_->evaluateFirstGrad(pos, dist_grad);
                Gradient_cx += 2 * (dist - dist_0_ ) * dist_grad(0) * A_.row(i);
                Gradient_cy += 2 * (dist - dist_0_ ) * dist_grad(1) * A_.row(i);
                Gradient_cz += 2 * (dist - dist_0_ ) * dist_grad(2) * A_.row(i);
            }
        }

        // fs and gradient of fs
        if (i > 0){
            pos      << state_x_(i),state_y_(i),state_z_(i);
            path_pos << path_x_(i), path_y_(i), path_z_(i);
            fs += pow((pos - path_pos).norm(),2);
            Gradient_sx += 2 * (pos(0) - path_pos(0)) * A_.row(i);
            Gradient_sy += 2 * (pos(1) - path_pos(1)) * A_.row(i);
            Gradient_sz += 2 * (pos(2) - path_pos(2)) * A_.row(i);
        }
    }
    // f_ mix and gradient mix
    f_ = alpha1_ * fu + alpha2_ * fc + alpha3_ * fs;
    Gradient_x_ = alpha1_ * Gradient_ux + alpha2_ * Gradient_cx + alpha3_ * Gradient_sx;
    Gradient_y_ = alpha1_ * Gradient_uy + alpha2_ * Gradient_cy + alpha3_ * Gradient_sy;
    Gradient_z_ = alpha1_ * Gradient_uz + alpha2_ * Gradient_cz + alpha3_ * Gradient_sz;
}

double low_mpc_planner::calPathRangeCost(double value){
    double range_cost;
    if (value < similary_lower_){
        range_cost = - pow((value - similary_lower_),3);
    } else if(value > similary_upper_) {
        range_cost = pow((value - similary_upper_), 3);
    } else{
        range_cost = 0.0;
    }
    return range_cost;
}

double low_mpc_planner::calPathRangeGrad(double value){
    double range_grad;
    if (value < similary_lower_){
        range_grad = -3 * pow((value - similary_lower_),2);
    } else if(value > similary_upper_) {
        range_grad = 3 * pow((value - similary_upper_), 2);
    } else{
        range_grad = 0.0;
    }
    return range_grad;
}

}    // namespace adaptive_planner