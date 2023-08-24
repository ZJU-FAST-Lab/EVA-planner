#include <path_searching/high_mpcc_optimizer.h>
#include <nlopt.hpp>

using namespace std;
using namespace Eigen;

namespace adaptive_planner{

void high_mpcc_optimizer::setParam(ros::NodeHandle& nh){
    nh.param("optimization/N", N_, 40);
    nh.param("optimization/K", K_, 3.0);
    nh.param("optimization/alpha_s", alpha_s_, 5.0);
    nh.param("optimization/alpha_p", alpha_p_, 0.001);
    nh.param("optimization/alpha_e", alpha_e_, 0.1);
    nh.param("optimization/alpha_c", alpha_c_, 10.0);
    nh.param("optimization/alpha_mu", alpha_mu_, 100.0);

    nh.param("optimization/alpha6", alpha6_, 100.0);
    nh.param("optimization/alpha7", alpha7_, 100.0);
    nh.param("optimization/alpha8", alpha8_, 100.0);
    nh.param("optimization/vel_min", vel_min_, 0.1);
    
    nh.param("multi_mpcc/Ts", Ts_, 0.05);
    nh.param("optimization/dist_0", dist_0_, 0.5);
    nh.param("optimization/dist_1", dist_1_, 2.0);
    nh.param("optimization/max_iteration_num", max_iteration_num_, 200);
    nh.param("optimization/max_iteration_time", max_iteration_time_, 0.05);

    nh.param("optimization/vel_u_lower", vel_u_lower_, -2.0);
    nh.param("optimization/vel_u_upper", vel_u_upper_, 2.0);
    nh.param("optimization/vel_w_lower", vel_w_lower_, -2.0);
    nh.param("optimization/vel_w_upper", vel_w_upper_, 2.0);

    alpha3_tmp_ = alpha_e_;

    // get the state matrix of the high mpc
    A_ = MatrixXd::Zero(N_,N_-1);
    A_mux_x_ = MatrixXd::Zero(N_,N_-1);
    A_mux_y_ = MatrixXd::Zero(N_,N_-1);
    B_ = VectorXd::Zero(N_);
    B_(0) = 1;

    input_mux_x_ = VectorXd::Zero(N_-1);
    input_mux_y_ = VectorXd::Zero(N_-1);

    for (int i = 1 ; i < N_ ; i++){
        for (int j = 0;j < i ; j++){
            A_(i,j) = Ts_;
        }
        B_(i) = 1;
    }

    resetInputInital();
}

void high_mpcc_optimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
    this->edt_map_ = env;
}

vector<vector<Vector3d>> high_mpcc_optimizer::mpccOptimizeTraj(Vector3d start_state,
                                                               vector<Vector2d> low_mpc_traj,
                                                               bool if_adaptive){
    // ros::Time tMpc = ros::Time::now();
    // if use the adaptive costfunction
    if (if_adaptive){
        alpha_e_ = alpha3_tmp_;
    } else{
        alpha_e_ = 0.0;
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
    vector<Vector3d>            traj_reference;

    for(int i = 0; i < N_ ; i++){
        Vector3d pos, vel, pos_ref;
        pos << state_x_(i), state_y_(i), state_theta_(i);
        if (i < N_ - 1) {
            vel << input_u_(i), input_w_(i), input_mu_(i);
        } else{
            vel << 0.0, 0.0, 0.0;
        }

        double t;
        t   =  state_mu_(i);
        pos_ref << muGetPosition(t), 0.0;
        traj_pos.push_back(pos);
        traj_vel.push_back(vel);
        traj_reference.push_back(pos_ref);

        /* ------------------------- debug start ------------------------- */

        /*if (i == 0){
            cout << "-----------mpcc vel-------------" << endl;
            cout <<"start vel  "<< start_state_(1,0) << "  " << start_state_(1,1)<< "  " << start_state_(1,2) << endl; 
        }
        cout << vel.transpose() << endl;*/

    }
    traj.push_back(traj_pos);
    traj.push_back(traj_vel);
    traj.push_back(traj_reference);

    /* ------------------------- debug start ------------------------- */
    // cout << "--------" << endl;
    // VectorXd mu_v = VectorXd::Zero(N_);
    // VectorXd delta_mu =  VectorXd::Zero(N_-1);
    // for(int i = 0; i < N_ ; i++){
    //     if(i>=1){
    //         delta_mu(i-1) = state_mu_(3*i) - state_mu_(3*i-3);
    //     }
    //     mu_v(i) = state_mu_(3*i+1);
    // }
    // cout << "delta_mu   " << delta_mu.transpose() << endl;
    // cout << "mu_v   " << mu_v.transpose() << endl;
    // debug end

    return traj;
}

void high_mpcc_optimizer::optimize(){
    /* initialize solver */
    iter_num_      = 0;
    min_cost_      = std::numeric_limits<double>::max();

    variable_num_  = dim_ * (N_ - 1);
    /* do optimization using NLopt slover */
//    nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_); // use the LD_LBFGS optimization method
    nlopt::opt opt(nlopt::algorithm::LD_MMA, variable_num_); // use the LD_LBFGS optimization method
    opt.set_min_objective(high_mpcc_optimizer::costFunction, this);
    opt.set_maxeval(max_iteration_num_);
    opt.set_maxtime(max_iteration_time_);
    opt.set_ftol_rel(1e-4);
    opt.set_vector_storage(16);

    // initial objective variables
    vector<double> q(variable_num_);
    for (int i = 0; i < (N_ - 1);i++){
        q[i] = input_u_(i);
    }
    for (int i = (N_ - 1); i < 2*(N_ - 1);i++){
        q[i] = input_w_(i-(N_ - 1));
    }
    for (int i = 2*(N_ - 1); i < 3*(N_ - 1);i++){
        q[i] = input_mu_(i-2*(N_ - 1));
    }
    double final_cost;
    nlopt::result result;
    try
    {
        result = opt.optimize(q, final_cost);
        cout << "High MPC Result:" << result << endl;
    }
    catch(std::exception& e)
    {
        ROS_WARN("[Optimization]: nlopt exception");
        cout << e.what() << endl;
        cout << "High MPC Result:" << result << endl;
    }
}

double high_mpcc_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data){
    auto opt = reinterpret_cast<high_mpcc_optimizer*>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);
  
    opt->iter_num_++;

    /* ------------------------- debug start ------------------------- */

//    cout << "---------------- iter: " << opt->iter_num_ <<  "---------------- " << endl;
//    cout << "cost :   " << cost << endl;
//    cout << "x:  " ;
//    for (int i = 0; i < x.size();i++){
//        cout << x.at(i) << " ";
//        if (i == opt->N_-1 || i == 2*(opt->N_-1) || i == 3*(opt->N_-1)){
//            cout << " " << endl;
//        }
//    }
//    cout << " " << endl;
//    cout << "grad:  ";
//    for (int i = 0; i < grad.size();i++){
//        cout << grad.at(i) << " ";
//        if (i == opt->N_-1 || i == 2*(opt->N_-1) || i == 3*(opt->N_-1)){
//            cout << " " << endl;
//        }
//    }
//    cout << " " << endl;

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
        input_u_(i) = x[i];
    }
    for (int i = (N_ - 1); i < 2*(N_ - 1);i++){
        input_w_(i-(N_ - 1)) = x[i];
    }
    for (int i = 2*(N_ - 1); i < 3*(N_ - 1);i++){
        input_mu_(i-2*(N_ - 1)) = x[i];
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
        grad[i] = Gradient_u_(i);
    }
    for (int i = (N_ - 1); i < 2*(N_ - 1);i++){
        grad[i] = Gradient_w_(i-(N_ - 1));
    }
    for (int i = 2*(N_ - 1); i < 3*(N_ - 1);i++){
        grad[i] = Gradient_mu_(i-2*(N_ - 1));
    }
}

void high_mpcc_optimizer::stateEquations(){
    // State theta
    state_theta_    = A_ * input_w_ + B_ * start_state_(2);
    // Inferred input in x-y axis: input_mux_x(t) = cos(theta(t)), input_mux_y(t) = sin(theta(t))
    // input_mux_x_ = state_theta_(seq(1, last)).array().cos();
    // input_mux_y_ = state_theta_(seq(1, last)).array().sin();

    auto m = state_theta_.size();
    input_mux_x_ = state_theta_.segment(1, m-1).array().cos();
    input_mux_y_ = state_theta_.segment(1, m-1).array().sin();


    // Obtain state and progress rate
    A_mux_x_        = A_.array().rowwise() * input_mux_x_.transpose().array();
    A_mux_y_        = A_.array().rowwise() * input_mux_y_.transpose().array();
    state_x_        = A_ * input_mux_x_.cwiseProduct(input_u_) + B_ * start_state_(0);
    state_y_        = A_ * input_mux_y_.cwiseProduct(input_u_) + B_ * start_state_(1);
    state_mu_       = A_ * input_mu_ + B_ * mu_start_state_;

    // Prepare matrices for gradient calculation
    Eigen::VectorXd x_mux_theta_, y_mux_theta_;
//    theta_mux_input_    = state_theta_(seq(0, last-1)) + input_w_ * Ts_;
    int tmp_x = 1;
    auto tmp = state_theta_.segment(0, m-1);
    tmp_x = 2;
    theta_mux_input_    = tmp  + input_w_ * Ts_;
    tmp_x = 3;

    x_mux_theta_        = -1.0 * theta_mux_input_.array().sin();
    y_mux_theta_        = theta_mux_input_.array().cos();
    A_mux_x_theta_      = A_.array().rowwise() * input_u_.cwiseProduct(x_mux_theta_).transpose().array();
    A_mux_y_theta_      = A_.array().rowwise() * input_u_.cwiseProduct(y_mux_theta_).transpose().array();
}

void high_mpcc_optimizer::setInitialState( Eigen::Vector3d& start_state,
                                           vector<Vector2d>& low_mpc_traj){
    // initial start state and system
    start_state_        = start_state;
    low_mpc_traj_       = low_mpc_traj;
    dim_                = 3;        // u, w, mu
    low_traj_num_       = static_cast<int>(low_mpc_traj_.size());
    low_traj_T_         = Ts_ * (low_traj_num_ - 1);
    mu_upper_           = low_traj_T_+ 1.0;
    // initial theta start state
    Vector2d pos_start;
    pos_start << start_state_(0), start_state_(1);
    double min_dist = inf;
    int    min_flag = 0;
    // find the nearest point in low_mpc_traj of the start postion
    for (int i = 0; i < low_traj_num_ ; i++){
        Vector2d pos;
        pos = low_mpc_traj_[i];
        double dist = (pos_start - pos).norm();
        if (dist < min_dist){
            min_dist = dist;
            min_flag = i;
        }
    }
    double t = Ts_ * min_flag;
    
    mu_start_state_ = t+0.4;
    mu_lower_ = t+0.4;
}

void high_mpcc_optimizer::resetInputInital(){
    // initial input
    input_u_ = VectorXd::Zero(N_-1);
    input_w_ = VectorXd::Zero(N_-1);
    input_mu_ = 20 * VectorXd::Ones(N_-1);
}

void high_mpcc_optimizer::setInitialSystem(){   
    // initial map size
    Vector2d map_origin, map_size;
    edt_map_->getMapRegion(map_origin, map_size);
    map_min_ = map_origin;
    map_max_ = map_origin + map_size;
}

Vector2d high_mpcc_optimizer::muGetPosition(double t){
    Vector2d P = Vector2d::Zero();
    if (t <= 0){
        P = low_mpc_traj_[0];
        return P;
    } else if (t >= low_traj_T_){
        P = low_mpc_traj_[low_traj_num_ - 1];
        return P;
    } else{
        int flag = floor(t / Ts_);
        Vector2d PL = low_mpc_traj_[flag];
        Vector2d PR = low_mpc_traj_[flag + 1];
        P(0) = PL(0) + (PR(0) - PL(0)) * ( t - Ts_ * flag) / Ts_;
        P(1) = PL(1) + (PR(1) - PL(1)) * ( t - Ts_ * flag) / Ts_;
        return P;
    }
}

void high_mpcc_optimizer::calCostFunctionandGradient(){
    // initial intermediate variables
    double fs, fp, fe, fc, fmu, fvel, fmu_vel;
    fs = fp = fe = fc = fmu = fvel = fmu_vel = 0.0;

    double mu;
    Vector2d pos, pos_mu;

    // Gradient container for f_s w.r.t. linear input input_u_, angular input input_w_, and progress rate input input_mu_
    VectorXd Gradient_su, Gradient_sw, Gradient_smu;
    Gradient_su = Gradient_sw = Gradient_smu = Eigen::VectorXd::Zero(N_ - 1);

    // Gradient container for f_p w.r.t. progress rate input input_mu_
    VectorXd Gradient_pmu;
    Gradient_pmu = Eigen::VectorXd::Zero(N_ - 1);

    // Gradient container for f_e w.r.t. linear input input_u_, angular input input_w_
    VectorXd Gradient_eu, Gradient_ew;
    Gradient_eu = Gradient_ew = Eigen::VectorXd::Zero(N_ - 1);

    // Gradient container for f_c w.r.t. linear input input_u_, angular input input_w_
    VectorXd Gradient_cu, Gradient_cw;
    Gradient_cu = Gradient_cw = Eigen::VectorXd::Zero(N_ - 1);

    // Gradient container for f_vel w.r.t. linear input input_u_, angular input input_w_
    VectorXd Gradient_velu, Gradient_velw;
    Gradient_velu = Gradient_velw = Eigen::VectorXd::Zero(N_ - 1);

    // Gradient container for f_mu w.r.t. progress rate input input_mu_
    VectorXd Gradient_mu;
    Gradient_mu = Eigen::VectorXd::Zero(N_ - 1);

    // Gradient container for f_mu_vel w.r.t. progress rate input input_mu_
    VectorXd Gradient_mu_vel;
    Gradient_mu_vel = Eigen::VectorXd::Zero(N_ - 1);
    
    // calculate the CostFunction and Gradient
    for(int i = 1;i < N_; i++){
        /* similarity between the traj and reference traj */
        // fs
        pos     << state_x_(i), state_y_(i);
        mu      =  state_mu_(i);
        pos_mu  =  muGetPosition(mu);
        fs      += pow((pos_mu - pos).norm() , 2);
        // gradient of fs
        double mu_L, mu_R;
        Eigen::Vector2d pos_mu_L, pos_mu_R;
        double delta_mu = 0.01;

        mu_R      = mu + delta_mu;
        mu_L      = mu - delta_mu;
        pos_mu_R  = muGetPosition(mu_R);
        pos_mu_L  = muGetPosition(mu_L);

        /* Finite Difference mu Calculation*/
        double grad_mu;
        grad_mu  =  (pos_mu(0) - pos(0)) * ( pos_mu_R(0) - pos_mu_L(0) ) / delta_mu;
        grad_mu  += (pos_mu(1) - pos(1)) * ( pos_mu_R(1) - pos_mu_L(1) ) / delta_mu;
        Gradient_su  += 2 * (pos(0) - pos_mu(0)) * A_mux_x_.row(i) + 2 * (pos(1) - pos_mu(1)) * A_mux_y_.row(i);
        Gradient_sw  += 2 * (pos(0) - pos_mu(0)) * A_mux_x_theta_.row(i - 1) + 2 * (pos(1) - pos_mu(1)) * A_mux_y_theta_.row(i - 1);
        Gradient_smu += grad_mu * A_.row(i);
        
        /* the cost of progress value */ 
        // fvthe a.k.a. f_p in paper
        fp              -= input_mu_(i - 1) * Ts_;
        // gradient of fvthe
        Gradient_pmu    -= A_.row(i) * Ts_;

        /* velocity direction dependent penalty */
        Vector2d vel, dist_grad, dist_grad2;
        double dist, beta, eta, p_eta_beta;
        vel << input_mux_x_(i - 1) * input_u_(i - 1), input_mux_y_(i - 1) * input_u_(i - 1);
        edt_map_->evaluateEDT(pos, dist);
        edt_map_->evaluateFirstGrad(pos, dist_grad);
        edt_map_->evaluateSecondGrad(pos, dist_grad2);
        if (abs(dist) < dist_1_){
            // fv and gradient of fv
            if (vel.norm() < 0.001 || dist_grad.norm() < 0.001){
                fe          += pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_ ,2);
                
                Gradient_eu += (2*(dist-dist_1_)*dist_grad(0) * A_mux_x_.row(i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(0) * (1.0-vel_min_) * A_mux_x_.row(i) / Ts_);
                Gradient_eu += (2*(dist-dist_1_)*dist_grad(1) * A_mux_y_.row(i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(1) * (1.0-vel_min_) * A_mux_y_.row(i) / Ts_);
                Gradient_ew += (2*(dist-dist_1_)*dist_grad(0) * A_mux_x_theta_.row(i - 1) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(0) * (1.0-vel_min_) * input_u_.cwiseProduct(-A_mux_y_.row(i).transpose()).transpose());
                Gradient_ew += (2*(dist-dist_1_)*dist_grad(1) * A_mux_y_theta_.row(i - 1) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(1) * (1.0-vel_min_) * input_u_.cwiseProduct(A_mux_x_.row(i).transpose()).transpose());
            } else{
                beta    =  vel.dot(dist_grad) / vel.norm() / dist_grad.norm();
                eta     =  2 / (1 + exp( K_ * beta));
                fe     += eta * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_ ,2);

                p_eta_beta       = 2 * K_ * exp(K_*beta) / pow(1+exp(K_*beta),2);
                // p_f_beta_ux
                double p_beta_vx            = (dist_grad(0)*pow(vel.norm(),2) - vel.dot(dist_grad)*vel(0)) / pow(vel.norm(),3) / dist_grad.norm();
                double p_beta_cx            = (vel(0)*pow(dist_grad.norm(),2) - vel.dot(dist_grad)*dist_grad(0)) / vel.norm() / pow(dist_grad.norm(),3);
                Eigen::VectorXd p_eta_ux    = p_eta_beta * (p_beta_vx * A_mux_x_.row(i) / Ts_ + p_beta_cx * dist_grad2(0) * A_mux_x_.row(i));
                Eigen::VectorXd p_eta_wx    = p_eta_beta * (p_beta_vx * input_u_.cwiseProduct(-A_mux_y_.row(i).transpose()) + p_beta_cx * dist_grad2(0) * A_mux_x_theta_.row(i - 1).transpose());
                // p_f_beta_uy
                double p_beta_vy            = (dist_grad(1)*pow(vel.norm(),2) - vel.dot(dist_grad)*vel(1)) / pow(vel.norm(),3) / dist_grad.norm();
                double p_beta_cy            = (vel(1)*pow(dist_grad.norm(),2) - vel.dot(dist_grad)*dist_grad(1)) / vel.norm() / pow(dist_grad.norm(),3);
                Eigen::VectorXd p_eta_uy    = p_eta_beta * (p_beta_vy * A_mux_y_.row(i) / Ts_ + p_beta_cy * dist_grad2(1) * A_mux_y_.row(i));
                Eigen::VectorXd p_eta_wy    = p_eta_beta * (p_beta_vy * input_u_.cwiseProduct(A_mux_x_.row(i).transpose()) + p_beta_cy * dist_grad2(1) * A_mux_y_theta_.row(i - 1).transpose());

                Gradient_eu += p_eta_ux.transpose() * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + eta * (2*(dist-dist_1_)*dist_grad(0) * A_mux_x_.row(i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(0) * (vel.norm()-vel_min_) / vel.norm() * A_mux_x_.row(i) / Ts_);
                Gradient_eu += p_eta_uy.transpose() * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + eta * (2*(dist-dist_1_)*dist_grad(1) * A_mux_y_.row(i) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(1) * (vel.norm()-vel_min_) / vel.norm() * A_mux_y_.row(i) / Ts_);
                Gradient_ew += p_eta_wx.transpose() * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + eta * (2*(dist-dist_1_)*dist_grad(0) * A_mux_x_theta_.row(i - 1) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(0) * (vel.norm()-vel_min_) / vel.norm() * input_u_.cwiseProduct(-A_mux_y_.row(i).transpose()).transpose());
                Gradient_ew += p_eta_wy.transpose() * pow(dist-dist_1_,2) * pow(vel.norm()-vel_min_,2) + eta * (2*(dist-dist_1_)*dist_grad(1) * A_mux_y_theta_.row(i - 1) * pow(vel.norm()-vel_min_,2) + 2 * pow(dist-dist_1_,2) * vel(1) * (vel.norm()-vel_min_) / vel.norm() * input_u_.cwiseProduct(A_mux_x_.row(i).transpose()).transpose());
            }
        }

//        cout <<"------------------" << i << "------------------"  << endl;
//        cout <<"Gradient_eu_ : " << Gradient_eu.transpose() << endl;
//        cout <<"Gradient_ew_ : " << Gradient_ew.transpose() << endl;

        /* the penalty of theta , which is used to prevent theta from exceeding the range  */
        /* theta dynamics is the same as position i.e. (x, y, z) */
        // ftheta
        fmu             += calRangeCost(mu, mu_lower_, mu_upper_);
        Gradient_mu     += calRangeGrad(mu, mu_lower_, mu_upper_) * A_.row(i);
        
        /* collision cost, basically the same as low mpc part */
        // fc and gradient of fc
        if (abs(dist) < dist_0_){
            fc += pow(dist - dist_0_,2);

            Gradient_cu += 2 * (dist - dist_0_ ) * dist_grad(0) * A_mux_x_.row(i);
            Gradient_cu += 2 * (dist - dist_0_ ) * dist_grad(1) * A_mux_y_.row(i);
            Gradient_cw += 2 * (dist - dist_0_ ) * dist_grad(0) * A_mux_x_theta_.row(i - 1);
            Gradient_cw += 2 * (dist - dist_0_ ) * dist_grad(1) * A_mux_y_theta_.row(i - 1);
        }
        
        /* minimum input velocity */
        /* velocity is the input, so there is no dynamics on it */
        // fvel
        if (i < N_ -1){
            Eigen::Vector2d input = Eigen::Vector2d(input_u_(i-1) ,input_w_(i-1));
            fvel                    += calRangeCost(input(0), vel_u_lower_, vel_u_upper_);
            fvel                    += calRangeCost(input(1), vel_w_lower_, vel_w_upper_);
            Gradient_velu(i-1) = calRangeGrad(input(0), vel_u_lower_, vel_u_upper_);
            Gradient_velw(i-1) = calRangeGrad(input(1), vel_w_lower_, vel_w_upper_);
        }

        /* the penalty of mu_v , which must > 0  */
        fmu_vel         += calvmuRangeCost(input_mu_(i - 1));
        Gradient_mu_vel += calvmuRangeGrad(input_mu_(i - 1)) * A_.row(i);
    }
    
    // f_ mix and gradient mix
    f_              = alpha_s_ * fs + alpha_p_ * fp + alpha_e_ * fe + alpha_c_ * fc + alpha_mu_ * fmu + alpha7_ * fvel + alpha8_ * fmu_vel;
    Gradient_u_     = alpha_s_ * Gradient_su + alpha_e_ * Gradient_eu + alpha_c_ * Gradient_cu + alpha7_ * Gradient_velu;
    Gradient_w_     = alpha_s_ * Gradient_sw + alpha_e_ * Gradient_ew + alpha_c_ * Gradient_cw + alpha7_ * Gradient_velw;
    Gradient_mu_    = alpha_s_ * Gradient_smu + alpha_p_ * Gradient_pmu + alpha_mu_ * Gradient_mu + alpha8_ * Gradient_mu_vel;

    /* ------------------------- debug start ------------------------- */
//    cout << "\n------------------" << iter_num_ << "------------------" << endl;
//    cout <<"f : " << f_ << endl;
//    cout <<"fe: " << alpha_e_ * fe << endl;
//    //cout <<"fjerk : " << alpha7_ *fjerk << endl;
//    cout <<"\nGradient_u_ : " << Gradient_u_.transpose() << endl;
//    cout <<"\nGradient_w_ : " << Gradient_w_.transpose() << endl;
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

double high_mpcc_optimizer::calvmuRangeCost(double value){
    double range_cost;

    if (value < 0.0){
        range_cost = pow(value,2);
    } else {
        range_cost = 0.0;
    }
    return range_cost;
}

double high_mpcc_optimizer::calvmuRangeGrad(double value){
    double range_grad;
    if (value < 0.0){
        range_grad = 2 * value;
    } else {
        range_grad = 0.0;
    }
    return range_grad;
}

}    // namespace adaptive_planner
