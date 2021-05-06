#include <path_searching/high_mpcc_optimizer.h>
#include <nlopt.hpp>

using namespace std;
using namespace Eigen;

namespace adaptive_planner{

void high_mpcc_optimizer::setParam(ros::NodeHandle& nh){
    nh.param("optimization/N", N_, -1);
    nh.param("optimization/K", K_, -1.0);
    nh.param("optimization/alpha1", alpha1_, -1.0);
    nh.param("optimization/alpha2", alpha2_, -1.0);
    nh.param("optimization/alpha3", alpha3_, -1.0);
    nh.param("optimization/alpha4", alpha4_, -1.0);
    nh.param("optimization/alpha5", alpha5_, -1.0);
    nh.param("optimization/alpha6", alpha6_, -1.0);
    nh.param("optimization/alpha7", alpha7_, -1.0);
    nh.param("optimization/alpha8", alpha8_, -1.0);
    nh.param("optimization/vel_min", vel_min_, -1.0);
    
    nh.param("multi_mpcc/Ts", Ts_, -1.0);
    nh.param("optimization/dist_0", dist_0_, -1.0);
    nh.param("optimization/dist_1", dist_1_, -1.0);
    nh.param("optimization/max_iteration_num", max_iteration_num_, -1);
    nh.param("optimization/max_iteration_time", max_iteration_time_, -1.0);

    nh.param("optimization/vel_lower", vel_lower_, -1.0);
    nh.param("optimization/vel_upper", vel_upper_, -1.0);
    nh.param("optimization/acc_lower", acc_lower_, -1.0);
    nh.param("optimization/acc_upper", acc_upper_, -1.0);
    nh.param("optimization/jerk_lower", jerk_lower_, -1.0);
    nh.param("optimization/jerk_upper", jerk_upper_, -1.0);

    alpha3_tmp_ = alpha3_;

    // build A_ and B_ 
    Matrix3d As = Matrix3d::Identity();
    Vector3d Bs = Vector3d::Zero();
    As(0,1) = Ts_;
    As(0,2) = Ts_ * Ts_ / 2 ;
    As(1,2) = Ts_;
    Bs(0)   = Ts_ * Ts_ * Ts_ / 6 ;
    Bs(1)   = Ts_ * Ts_ / 2 ;
    Bs(2)   = Ts_;
    MatrixPower<Matrix3d> Apow(As);

    A_ = MatrixXd::Zero(3 * N_, N_ -1);
    B_ = MatrixXd::Zero(3 * N_, 3);
    B_.block(0,0,3,3) = Matrix3d::Identity();
    for (int i = 1 ; i < N_ ; i++){
        for (int j = 0; j< i; j++){
            A_.block(3*i,j,3,1) = Apow(i-j-1) * Bs;
        }
        B_.block(3*i,0,3,3) = Apow(i);
    }

    resetInputInital();
}

void high_mpcc_optimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
    this->edt_map_ = env;
}

vector<vector<Vector3d>> high_mpcc_optimizer::mpccOptimizeTraj( Matrix3d start_state,
                                                                vector<Vector3d> low_mpc_traj,
                                                                bool if_adaptive){
    // ros::Time tMpc = ros::Time::now();
    // if use the adaptive costfunction
    if (if_adaptive){
        alpha3_ = alpha3_tmp_;
    } else{
        alpha3_ = 0.0;
    }
    // set initial state
    setInitialState(start_state, low_mpc_traj);
    // set initial system matrix and control input
    setInitialSystem();

    // use the NLopt optimizer
    optimize();

    // return the optimized path
    vector<vector<Vector3d>>    traj;
    vector<Vector3d>            traj_pos;
    vector<Vector3d>            traj_vel;
    vector<Vector3d>            traj_acc;
    vector<Vector3d>            traj_reference;

    for(int i = 0; i < N_ ; i++){
        Vector3d pos, vel, acc, pos_ref;
        pos << state_x_(3*i), state_y_(3*i), state_z_(3*i);
        vel << state_x_(3*i+1), state_y_(3*i+1), state_z_(3*i+1);
        acc << state_x_(3*i+2), state_y_(3*i+2), state_z_(3*i+2);
        double t;
        t   =  state_theta_(3*i);
        pos_ref = thetaGetPosition(t);
        traj_pos.push_back(pos);
        traj_vel.push_back(vel);
        traj_acc.push_back(acc);
        traj_reference.push_back(pos_ref);
        // debug 
        /*if (i == 0){
            cout << "-----------mpcc vel-------------" << endl;
            cout <<"start vel  "<< start_state_(1,0) << "  " << start_state_(1,1)<< "  " << start_state_(1,2) << endl; 
        }
        cout << vel.transpose() << endl;*/
    }
    traj.push_back(traj_pos);
    traj.push_back(traj_vel);
    traj.push_back(traj_acc);
    traj.push_back(traj_reference);

    /* debug start */
    // cout << "--------" << endl;
    // VectorXd theta_v = VectorXd::Zero(N_);
    // VectorXd delta_theta =  VectorXd::Zero(N_-1);
    // for(int i = 0; i < N_ ; i++){
    //     if(i>=1){
    //         delta_theta(i-1) = state_theta_(3*i) - state_theta_(3*i-3);
    //     }
    //     theta_v(i) = state_theta_(3*i+1);
    // }
    // cout << "delta_theta   " << delta_theta.transpose() << endl;
    // cout << "theta_v   " << theta_v.transpose() << endl;
    // debug end

    return traj;
}

void high_mpcc_optimizer::optimize(){
    /* initialize solver */
    iter_num_      = 0;
    min_cost_      = std::numeric_limits<double>::max();

    variable_num_  = dim_ * (N_ - 1);
    /* do optimization using NLopt slover */
    nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_); // use the LD_LBFGS optimization method
    opt.set_min_objective(high_mpcc_optimizer::costFunction, this);
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
    for (int i = 3*(N_ - 1); i < 4*(N_ - 1);i++){
        q[i] = input_theta_(i-3*(N_ - 1));
    }
    double final_cost;
    try
    {
        nlopt::result result = opt.optimize(q, final_cost);
    }
    catch(std::exception& e)
    {
        //ROS_WARN("[Optimization]: nlopt exception");
        //cout << e.what() << endl;
    }
}

double high_mpcc_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data){
    high_mpcc_optimizer* opt = reinterpret_cast<high_mpcc_optimizer*>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);
  
    opt->iter_num_++;

    //debug start
    /*cout << "---------------- iter: " << opt->iter_num_ <<  "---------------- " << endl;
    cout << "cost :   " << cost << endl;
    cout << "x:  " ;
    for (int i = 0; i < x.size();i++){
        cout << x.at(i) << " ";
        if (i == opt->N_-1 || i == 2*(opt->N_-1) || i == 3*(opt->N_-1)){
            cout << " " << endl;
        }
    }
    cout << " " << endl;
    cout << "grad:  ";
    for (int i = 0; i < grad.size();i++){
        cout << grad.at(i) << " ";
        if (i == opt->N_-1 || i == 2*(opt->N_-1) || i == 3*(opt->N_-1)){
            cout << " " << endl;
        }
    }
    cout << " " << endl;*/
    // debug end

    /* save the min cost result */
    if (cost < opt->min_cost_) {
        opt->min_cost_      = cost;
        opt->best_variable_ = x;
    }
    return cost;
}

void high_mpcc_optimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,  double& f_combine){
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
    for (int i = 3*(N_ - 1); i < 4*(N_ - 1);i++){
        input_theta_(i-3*(N_ - 1)) = x[i];
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
    for (int i = 3*(N_ - 1); i < 4*(N_ - 1);i++){
        grad[i] = Gradient_theta_(i-3*(N_ - 1));
    }
}

void high_mpcc_optimizer::stateEquations(){
    state_x_     = A_ * input_x_ + B_ * start_state_.col(0);
    state_y_     = A_ * input_y_ + B_ * start_state_.col(1);
    state_z_     = A_ * input_z_ + B_ * start_state_.col(2);
    state_theta_ = A_ * input_theta_ + B_ * theta_start_state_;
}

void high_mpcc_optimizer::setInitialState( Matrix3d start_state,
                                           vector<Vector3d> low_mpc_traj){
    // initial start state and system
    start_state_        = start_state;
    low_mpc_traj_       = low_mpc_traj;
    dim_                = 4;        // x, y, z, theta
    low_traj_num_       = low_mpc_traj_.size();
    low_traj_T_         = Ts_ * (low_traj_num_ - 1);
    theta_upper_        = low_traj_T_+ 1.0;
    // initial theta start state
    Vector3d pos_start;
    pos_start << start_state_(0,0), start_state_(0,1), start_state_(0,2);
    double min_dist = inf;
    int    min_flag = 0;
    // find the nearest point in low_mpc_traj of the start postion
    for (int i = 0; i < low_traj_num_ ; i++){
        Vector3d pos;
        pos = low_mpc_traj_[i];
        double dist = (pos_start - pos).norm();
        if (dist < min_dist){
            min_dist = dist;
            min_flag = i;
        }
    }
    double t = Ts_ * min_flag;
    // theta_start_state_ << t+0.4, 0.0, 0.0;
    // theta_lower_ = t+0.4;
    
    theta_start_state_ << t+0.4, 0.0, 0.0;
    theta_lower_ = t+0.4;
}

void high_mpcc_optimizer::resetInputInital(){
    // initial input
    input_x_ = VectorXd::Zero(N_-1);
    input_y_ = VectorXd::Zero(N_-1);
    input_z_ = VectorXd::Zero(N_-1);
    input_theta_ = 20 * VectorXd::Ones(N_-1);
}

void high_mpcc_optimizer::setInitialSystem(){   
    // initial map size
    Vector3d map_origin, map_size;
    edt_map_->getMapRegion(map_origin, map_size);
    map_min_ = map_origin;
    map_max_ = map_origin + map_size;
}

Vector3d high_mpcc_optimizer::thetaGetPosition(double t){
    Vector3d P = Vector3d::Zero();
    if (t <= 0){
        P = low_mpc_traj_[0];
        return P;
    } else if (t >= low_traj_T_){
        P = low_mpc_traj_[low_traj_num_ - 1];
        return P;
    } else{
        int flag = floor(t / Ts_);
        Vector3d PL = low_mpc_traj_[flag];
        Vector3d PR = low_mpc_traj_[flag + 1];
        P(0) = PL(0) + (PR(0) - PL(0)) * ( t - Ts_ * flag) / Ts_;
        P(1) = PL(1) + (PR(1) - PL(1)) * ( t - Ts_ * flag) / Ts_;
        P(2) = PL(2) + (PR(2) - PL(2)) * ( t - Ts_ * flag) / Ts_;
        return P;
    }
}

Vector3d high_mpcc_optimizer::getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;
    int _poly_num1D = (int)polyCoeff.cols()/3;
    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }
    return ret;
}

void high_mpcc_optimizer::calCostFunctionandGradient(){
    // initial intermediate variables
    double fs, fvthe, fv, fc, ftheta, fvelacc, fjerk, ftheta_v;
    fs = fvthe = fv = fc = ftheta = fvelacc = fjerk = ftheta_v = 0.0;
    Vector3d pos, vel, acc, pos_theta, pos_theta1, pos_theta2, dist_grad, dist_grad2;
    double theta, theta1, theta2, grad_theta, delta_theta, dist;
    double beta, f_beta, p_f_beta_beta, p_beta_vx, p_beta_vy, p_beta_vz, p_beta_cx, p_beta_cy, p_beta_cz;
    VectorXd p_f_beta_ux, p_f_beta_uy, p_f_beta_uz;
    Vector3d input, input_next;
    delta_theta = 0.01;

    VectorXd Gradient_sx, Gradient_sy, Gradient_sz, Gradient_stheta;
    VectorXd Gradient_vthe;
    VectorXd Gradient_vx, Gradient_vy, Gradient_vz;
    VectorXd Gradient_cx, Gradient_cy, Gradient_cz;
    VectorXd Gradient_the;
    VectorXd Gradient_velx, Gradient_vely, Gradient_velz, Gradient_accx, Gradient_accy, Gradient_accz;
    VectorXd Gradient_jerkx, Gradient_jerky, Gradient_jerkz;
    VectorXd Gradient_thetav;
    Gradient_sx   = Gradient_sy = Gradient_sz = Gradient_stheta = VectorXd::Zero(N_-1);
    Gradient_vthe = VectorXd::Zero(N_-1);
    Gradient_vx   = Gradient_vy = Gradient_vz = VectorXd::Zero(N_-1);
    Gradient_cx   = Gradient_cy = Gradient_cz = VectorXd::Zero(N_-1);
    Gradient_the  = VectorXd::Zero(N_-1);
    Gradient_velx = Gradient_vely = Gradient_velz = VectorXd::Zero(N_-1);
    Gradient_accx = Gradient_accy = Gradient_accz = VectorXd::Zero(N_-1);
    Gradient_jerkx= Gradient_jerky = Gradient_jerkz = VectorXd::Zero(N_-1);
    Gradient_thetav = VectorXd::Zero(N_-1);
    
    // calculate the CostFunction and Gradient
    for(int i = 1;i < N_; i++){
        /* similarity between the traj and reference traj */
        // fs
        pos       << state_x_(3*i), state_y_(3*i), state_z_(3*i); 
        theta     =  state_theta_(3*i);
        pos_theta =  thetaGetPosition(theta);
        fs        += pow((pos_theta - pos).norm() , 2);
        // gradient of fs
        theta1      = theta + delta_theta;
        theta2      = theta - delta_theta;
        pos_theta1  = thetaGetPosition(theta1);
        pos_theta2  = thetaGetPosition(theta2);
        grad_theta  =  (pos_theta(0) - pos(0)) * ( pos_theta1(0) - pos_theta2(0) ) / delta_theta;
        grad_theta  += (pos_theta(1) - pos(1)) * ( pos_theta1(1) - pos_theta2(1) ) / delta_theta;
        grad_theta  += (pos_theta(2) - pos(2)) * ( pos_theta1(2) - pos_theta2(2) ) / delta_theta;
        Gradient_sx     += 2 * (pos(0) - pos_theta(0)) * A_.row(3*i);
        Gradient_sy     += 2 * (pos(1) - pos_theta(1)) * A_.row(3*i);
        Gradient_sz     += 2 * (pos(2) - pos_theta(2)) * A_.row(3*i);
        Gradient_stheta += grad_theta * A_.row(3*i);
        
        /* the cost of progress value */ 
        // fvthe
        fvthe          += state_theta_(3*i + 1);
        // gradient of fvthe
        Gradient_vthe  -= A_.row(3*i + 1) * Ts_;

        /* velocity directioin depend panelty */
        vel << state_x_(3*i+1), state_y_(3*i+1), state_z_(3*i+1);
        edt_map_->evaluateEDT(pos, dist);
        edt_map_->evaluateFirstGrad(pos, dist_grad);
        edt_map_->evaluateSecondGrad(pos, dist_grad2);
        if (abs(dist) < dist_1_){
            // fv and gradient of fv
            if (vel.norm() < 0.001 || dist_grad.norm() < 0.001){
                fv     += pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_ ,2);
                
                Gradient_vx += (2*(dist-dist_1_)*dist_grad(0) * A_.row(3*i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(0) * (vel.norm()-vel_min_) / vel.norm() * A_.row(3*i+1));
                Gradient_vy += (2*(dist-dist_1_)*dist_grad(1) * A_.row(3*i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(1) * (vel.norm()-vel_min_) / vel.norm() * A_.row(3*i+1));
                Gradient_vz += (2*(dist-dist_1_)*dist_grad(2) * A_.row(3*i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(2) * (vel.norm()-vel_min_) / vel.norm() * A_.row(3*i+1));
            } else{
                beta   =  vel.dot(dist_grad) / vel.norm() / dist_grad.norm();
                f_beta =  2 / (1 + exp( K_ * beta));
                fv     += f_beta * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_ ,2);

                p_f_beta_beta = 2 * K_ * exp(K_*beta) / pow(1+exp(K_*beta),2);
                // p_f_beta_ux
                p_beta_vx = (dist_grad(0)*pow(vel.norm(),2) - vel.dot(dist_grad)*vel(0)) / pow(vel.norm(),3) / dist_grad.norm(); 
                p_beta_cx = (vel(0)*pow(dist_grad.norm(),2) - vel.dot(dist_grad)*dist_grad(0)) / vel.norm() / pow(dist_grad.norm(),3);
                p_f_beta_ux = p_f_beta_beta * (p_beta_vx * A_.row(3*i + 1) + p_beta_cx * dist_grad2(0) * A_.row(3*i));
                // p_f_beta_uy
                p_beta_vy = (dist_grad(1)*pow(vel.norm(),2) - vel.dot(dist_grad)*vel(1)) / pow(vel.norm(),3) / dist_grad.norm(); 
                p_beta_cy = (vel(1)*pow(dist_grad.norm(),2) - vel.dot(dist_grad)*dist_grad(1)) / vel.norm() / pow(dist_grad.norm(),3);
                p_f_beta_uy = p_f_beta_beta * (p_beta_vy * A_.row(3*i + 1) + p_beta_cy * dist_grad2(1) * A_.row(3*i));
                // p_f_beta_uz
                p_beta_vz = (dist_grad(2)*pow(vel.norm(),2) - vel.dot(dist_grad)*vel(2)) / pow(vel.norm(),3) / dist_grad.norm(); 
                p_beta_cz = (vel(2)*pow(dist_grad.norm(),2) - vel.dot(dist_grad)*dist_grad(2)) / vel.norm() / pow(dist_grad.norm(),3);
                p_f_beta_uz = p_f_beta_beta * (p_beta_vz * A_.row(3*i + 1) + p_beta_cz * dist_grad2(2) * A_.row(3*i));

                Gradient_vx += p_f_beta_ux * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + f_beta * (2*(dist-dist_1_)*dist_grad(0) * A_.row(3*i).transpose() * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(0) * (vel.norm()-vel_min_) / vel.norm() * A_.row(3*i+1).transpose());
                Gradient_vy += p_f_beta_uy * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + f_beta * (2*(dist-dist_1_)*dist_grad(1) * A_.row(3*i).transpose() * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(1) * (vel.norm()-vel_min_) / vel.norm() * A_.row(3*i+1).transpose());
                Gradient_vz += p_f_beta_uz * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + f_beta * (2*(dist-dist_1_)*dist_grad(2) * A_.row(3*i).transpose() * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(2) * (vel.norm()-vel_min_) / vel.norm() * A_.row(3*i+1).transpose());
            }
        }
        
        /* collision cost */
        // fc and gradient of fc
        if (abs(dist) < dist_0_){
            fc += pow(dist - dist_0_,2);

            Gradient_cx += 2 * (dist - dist_0_ ) * dist_grad(0) * A_.row(3*i);
            Gradient_cy += 2 * (dist - dist_0_ ) * dist_grad(1) * A_.row(3*i);
            Gradient_cz += 2 * (dist - dist_0_ ) * dist_grad(2) * A_.row(3*i);
        }
        
        /* the penalty of theta , which is used to prevent theta from exceeding the range  */ 
        // ftheta
        ftheta       += calRangeCost(theta, theta_lower_, theta_upper_);
        Gradient_the += calRangeGrad(theta, theta_lower_, theta_upper_) * A_.row(3*i);

        /* the penalty of vel and acc, which is used to prevent vel and acc from exceeding the range */
        // fvelacc
        acc << state_x_(3*i+2), state_y_(3*i+2), state_z_(3*i+2);
        fvelacc  += calRangeCost(vel(0), vel_lower_, vel_upper_);
        fvelacc  += calRangeCost(vel(1), vel_lower_, vel_upper_);
        fvelacc  += calRangeCost(vel(2), vel_lower_, vel_upper_);
        fvelacc  += calRangeCost(acc(0), acc_lower_, acc_upper_);
        fvelacc  += calRangeCost(acc(1), acc_lower_, acc_upper_);
        fvelacc  += calRangeCost(acc(2), acc_lower_, acc_upper_);
        // gradient of fvelacc
        Gradient_velx += calRangeGrad(vel(0), vel_lower_, vel_upper_) * A_.row(3*i+1);
        Gradient_vely += calRangeGrad(vel(1), vel_lower_, vel_upper_) * A_.row(3*i+1);
        Gradient_velz += calRangeGrad(vel(2), vel_lower_, vel_upper_) * A_.row(3*i+1);
        Gradient_accx += calRangeGrad(acc(0), acc_lower_, acc_upper_) * A_.row(3*i+2);
        Gradient_accy += calRangeGrad(acc(1), acc_lower_, acc_upper_) * A_.row(3*i+2);
        Gradient_accz += calRangeGrad(acc(2), acc_lower_, acc_upper_) * A_.row(3*i+2); 
        
        /* minimum input jerk */
        // fjerk
        if (i < N_ -1){
            input  << input_x_(i-1) ,input_y_(i-1), input_z_(i-1);
            // fjerk += pow(input.norm(),2);
            // Gradient_jerkx(i) = 2 * input(0);
            // Gradient_jerky(i) = 2 * input(1);
            // Gradient_jerkz(i) = 2 * input(2);
            fjerk += calRangeCost(input(0), jerk_lower_, jerk_upper_);
            fjerk += calRangeCost(input(1), jerk_lower_, jerk_upper_);
            fjerk += calRangeCost(input(2), jerk_lower_, jerk_upper_);
            Gradient_jerkx(i-1) = calRangeGrad(input(0), jerk_lower_, jerk_upper_);
            Gradient_jerky(i-1) = calRangeGrad(input(1), jerk_lower_, jerk_upper_);
            Gradient_jerkz(i-1) = calRangeGrad(input(2), jerk_lower_, jerk_upper_);
        }       

        /* the penalty of theta_v , which must > 0  */ 
        ftheta_v += calvthetaRangeCost(state_theta_(3*i+1));
        Gradient_thetav += calvthetaRangeGrad(state_theta_(3*i+1)) * A_.row(3*i+1);
    }
    
    // f_ mix and gradient mix
    f_              = alpha1_ * fs + alpha2_ * fvthe + alpha3_ * fv + alpha4_ * fc + alpha5_ * ftheta + alpha6_ * fvelacc + alpha7_ * fjerk + alpha8_ * ftheta_v;
    Gradient_x_     = alpha1_ * Gradient_sx + alpha3_ *  Gradient_vx + alpha4_ * Gradient_cx + alpha6_ * (Gradient_velx + Gradient_accx) + alpha7_ * Gradient_jerkx;
    Gradient_y_     = alpha1_ * Gradient_sy + alpha3_ *  Gradient_vy + alpha4_ * Gradient_cy + alpha6_ * (Gradient_vely + Gradient_accy) + alpha7_ * Gradient_jerky;
    Gradient_z_     = alpha1_ * Gradient_sz + alpha3_ *  Gradient_vz + alpha4_ * Gradient_cz + alpha6_ * (Gradient_velz + Gradient_accz) + alpha7_ * Gradient_jerkz;
    Gradient_theta_ = alpha1_ * Gradient_stheta + alpha2_ * Gradient_vthe + alpha5_ * Gradient_the + alpha8_ * Gradient_thetav;

    // debug
    /*cout <<"------------------" << endl;
    cout <<"f : " << f_ << endl;
    cout <<"fv: " << alpha3_ *fv << endl;
    //cout <<"fjerk : " << alpha7_ *fjerk << endl;
    cout <<"Gradient_x_ : " << Gradient_x_.transpose() << endl;
    cout <<"Gradient_vx : " << alpha3_*Gradient_vx.transpose() << endl;*/
    //cout <<"Gradient_jerkx : " << alpha7_*Gradient_jerkx.transpose() << endl;
    
}

double high_mpcc_optimizer::calRangeCost(double value, double lower,double upper){
    double range_cost;
    // if (value < lower){
    //     range_cost = -pow((value - lower),3);
    // } else if (value > upper){
    //     range_cost = pow((value - upper),3);
    // } else {
    //     range_cost = 0.0;
    // }
    if (value < lower){
        range_cost = pow((value - lower),2);
    } else if (value > upper){
        range_cost = pow((value - upper),2);
    } else {
        range_cost = 0.0;
    }
    return range_cost;
}

double high_mpcc_optimizer::calRangeGrad(double value, double lower,double upper){
    double range_grad;
    // if (value < lower){
    //     range_grad = -3 * pow((value - lower),2);
    // } else if (value > upper){
    //     range_grad = 3 * pow((value - upper),2);
    // } else {
    //     range_grad = 0.0;
    // }
    if (value < lower){
        range_grad = 2 * (value - lower);
    } else if (value > upper){
        range_grad = 2 * (value - upper);
    } else {
        range_grad = 0.0;
    }
    return range_grad;
}

//
double high_mpcc_optimizer::calvthetaRangeCost(double value){
    double range_cost;

    if (value < 0.0){
        range_cost = pow(value,2);
    } else {
        range_cost = 0.0;
    }
    return range_cost;
}

double high_mpcc_optimizer::calvthetaRangeGrad(double value){
    double range_grad;
    if (value < 0.0){
        range_grad = 2 * value;
    } else {
        range_grad = 0.0;
    }
    return range_grad;
}

}    // namespace adaptive_planner