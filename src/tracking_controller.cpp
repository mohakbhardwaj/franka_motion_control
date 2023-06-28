#include <stdexcept>
#include "tracking_controller.h"


TrackingController::TrackingController(ros::NodeHandle* nh, ros::NodeHandle* pnh, std::string robot_ip):
                        FrankaController(nh, pnh, robot_ip){

    tau_d_error_.setZero();
    tau_d_coriolis_.setZero(); 
    tau_d_inertia_.setZero();
    tau_d_calculated_.setZero();
    delta_cmd_tau_.setZero();
    prev_cmd_tau_.setZero();

    integral_err_.setZero();
    i_min_.setZero();
    i_thresh_.setZero();  
    sat_tau_.setZero();
      
    
    Km_.setZero();
    Kp_.setZero();
    Kd_.setZero();
    Ki_.setZero();
    gains_set_ = false;

    cmd_dt_ = 0.001;
}

void TrackingController::initialize_control_gains(){
    std::vector<double> Kp, Kd, Km, Ki, i_min, i_thresh, sat_tau;
    pnh_.getParam("Kp", Kp);
    pnh_.getParam("Kd", Kd);
    pnh_.getParam("Km", Km);
    pnh_.getParam("Ki", Ki);
    pnh_.getParam("i_min", i_min);
    pnh_.getParam("i_thresh", i_thresh);
    pnh_.getParam("saturate_tau", sat_tau);

    pnh_.getParam("alpha_q", alpha_q_);
    pnh_.getParam("alpha_dq", alpha_dq_);

    for(size_t i = 0; i < 7; ++i){
        Kp_[i] = Kp[i];
        Kd_[i] = Kd[i];
        Km_[i] = Km[i];   
        Ki_[i] = Ki[i];
        i_min_[i] = i_min[i];
        i_thresh_[i] = i_thresh[i];
        sat_tau_[i] = sat_tau[i];
    }
    
    gains_set_ = true;
}

void TrackingController::command_loop(){
    if (!gains_set_){
        throw std::runtime_error("Control gains not set!!");
    }
    while(ros::ok()){
        robot_.control([&](const franka::RobotState& robot_state, franka::Duration period) 
                                            -> franka::Torques {
            return torque_controller_callback(robot_state, period);
        }
        );
    }

}

franka::Torques TrackingController::torque_controller_callback(const franka::RobotState& robot_state, franka::Duration period){    
    time_ += period.toSec();
    // publishRobotState(robot_state);
    curr_q_ = Vector7d(robot_state.q.data());
    // curr_dq_ = Vector7d(robot_state.dq.data());

    if(time_ == 0.0){
        //Initial state to hold while waiting for command
        q_des_cmd_ = curr_q_;
        dq_des_cmd_ = curr_dq_;
        ddq_des_cmd_.setZero();
        curr_q_bel_= curr_q_;
        prev_q_bel_ = curr_q_;
        curr_dq_bel_ = curr_dq_;
        integral_err_.setZero();
    }
    else{
        //filter state
        curr_q_bel_ = alpha_q_ * curr_q_ + (1.0 - alpha_q_) * curr_q_bel_;

        curr_dq_ = (curr_q_bel_ - prev_q_bel_) / cmd_dt_;
        for(size_t i=0; i < 7; ++i){
            if(std::abs(curr_dq_[i]) <= 0.05){
                curr_dq_[i] = 0.0;
            }
        }

        curr_dq_bel_ = alpha_dq_ * curr_dq_ + (1.0 - alpha_dq_) * curr_dq_bel_;

        for(size_t i=0; i < 7; ++i){
            if(std::abs(curr_dq_bel_[i]) <= 0.001){
                curr_dq_bel_[i] = 0.0;
            }
        }

    }

    publishRobotState(curr_q_bel_, curr_dq_bel_);
    prev_q_bel_ = curr_q_bel_;


    Mat7d inertia_matrix = Mat7d(model_.mass(robot_state).data()); 
    tau_d_coriolis_ = Vector7d(model_.coriolis(robot_state).data()); //coriolis torque from current state

    if(command_pub_started_){

        // //Filter state
        // curr_q_bel_ = alpha_q_ * curr_q_ + (1.0 - alpha_q_) * curr_q_bel_;

        // // curr_dq_bel_ = clip
        // curr_dq_bel_ = alpha_dq_ * curr_dq_ + (1.0 - alpha_dq_) * curr_dq_bel_;

        //update desired command
        q_des_cmd_ = curr_q_des_;
        dq_des_cmd_ = curr_dq_des_;
        ddq_des_cmd_ = curr_ddq_des_;

    }

    tau_d_inertia_ = inertia_matrix * ddq_des_cmd_;
    tau_d_inertia_ = Km_.cwiseProduct(tau_d_inertia_);

    curr_q_err_= q_des_cmd_ - curr_q_bel_;
        
    // compute integral error
    integral_err_ += curr_q_err_ * cmd_dt_;
        
    for(int i=0; i < 7; ++i){
        if(abs(curr_q_err_[i]) < i_min_[i]){
            integral_err_[i] = 0.0;
        }
    
        if(abs(integral_err_[i]) > i_thresh_[i]){
            integral_err_[i] = copysign(i_thresh_[i], integral_err_[i]);
        }
    
        // if(abs(dq_des_cmd_[i]) > min_vel_[i] && raw_tau_[i] > min_tau_[i]){
        //     tau_d_min_[i] = copysign(offset_tau_[i], dq_des_cmd_[i]);
        // }
        
    }

    tau_d_error_ = Kp_.cwiseProduct(curr_q_err_) +  Kd_.cwiseProduct(dq_des_cmd_ - curr_dq_bel_) + Ki_.cwiseProduct(integral_err_);

    tau_d_calculated_ =  tau_d_inertia_ + tau_d_error_ + tau_d_coriolis_;

    // for(int i = 0; i < 7; ++i){
    //     std::cout << integral_err_[i] << " ";
    // }
    // std::cout << integral_err_[6] << "\n";

    saturate_torque(tau_d_calculated_);

    Eigen::VectorXd::Map(&tau_d_calculated_arr_[0], 7) = tau_d_calculated_;
    
    // std::array<double, 7> tau_d_rate_limited =
    //         franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated_arr, robot_state.tau_J_d);
    
    // if(!ros::ok()){
    //     ROS_INFO("Ros shutdown");
    // }
    ros::spinOnce();
    return tau_d_calculated_arr_; //tau_d_rate_limited;
}

void TrackingController::saturate_torque(const Vector7d& tau)
{
  delta_cmd_tau_ = tau - prev_cmd_tau_;
  
  for(int i=0; i < 7; ++i){

    if(std::abs(delta_cmd_tau_[i]) > sat_tau_[i]){
      tau_d_calculated_[i] = prev_cmd_tau_[i] + copysign(sat_tau_[i], delta_cmd_tau_[i]);
    }
  }

  prev_cmd_tau_ = tau_d_calculated_;
}