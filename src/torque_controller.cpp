#include "torque_controller.h"


TorqueController::TorqueController(ros::NodeHandle* nodehandle, std::string robot_ip):
                        FrankaController(nodehandle, robot_ip){

    tau_d_error_.setZero();
    tau_d_coriolis_.setZero(); 
    tau_d_inertia_.setZero();
    tau_d_calculated_.setZero();

}


franka::Torques TorqueController::torque_controller_callback(const franka::RobotState& robot_state, franka::Duration period){
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
    }
    else{
        //filter state
        curr_q_bel_ = alpha_q_ * curr_q_ + (1.0 - alpha_q_) * curr_q_bel_;

        curr_dq_ = (curr_q_bel_ - prev_q_bel_) / 0.001;
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
    tau_d_inertia_ = Pf_.cwiseProduct(tau_d_inertia_);

    tau_d_error_ = P_.cwiseProduct(q_des_cmd_ - curr_q_bel_) +  D_.cwiseProduct(dq_des_cmd_ - curr_dq_bel_);
    tau_d_calculated_ =  tau_d_inertia_ + tau_d_error_ + tau_d_coriolis_;

    Eigen::VectorXd::Map(&tau_d_calculated_arr_[0], 7) = tau_d_calculated_;
    
    // std::array<double, 7> tau_d_rate_limited =
    //         franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated_arr, robot_state.tau_J_d);
    
    // if(!ros::ok()){
    //     ROS_INFO("Ros shutdown");
    // }
    ros::spinOnce();
    return tau_d_calculated_arr_; //tau_d_rate_limited;
}

void TorqueController::control_loop(){
    
    while(ros::ok()){
        robot_.control([&](const franka::RobotState& robot_state, franka::Duration period) 
                                            -> franka::Torques {
            return torque_controller_callback(robot_state, period);
        }
        );
    }

}
