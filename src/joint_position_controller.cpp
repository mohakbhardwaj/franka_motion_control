// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun example_ros_class example_ros_class
// rostopic echo exampleMinimalPubTopic
// rostopic pub -r 4 exampleMinimalSubTopic std_msgs/Float32 2.0
// rosservice call exampleMinimalService 1


// this header incorporates all the necessary #include files and defines the class "JointPositionController"
#include "joint_position_controller.h"


// JointPositionController::JointPositionController(ros::NodeHandle* nodehandle, franka::Robot& robot):

JointPositionController::JointPositionController(ros::NodeHandle* nodehandle, std::string robot_ip):
                        nh_(*nodehandle), robot_(robot_ip), model_(robot_.loadModel()) //std::string robot_ip, robot_(robot_ip), 
{   ROS_INFO("in class constructor of JointPositionController");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    // initializeServices();
    // robot_ip_ = robot_ip;
    curr_q_.setZero();
    curr_dq_.setZero();
    curr_q_des_.setZero();
    curr_dq_des_.setZero();
    curr_ddq_des_.setZero();
    delta_q_.setZero();
    curr_q_bel_.setZero();
    curr_dq_bel_.setZero();
    
    // robot_ =
    // model_ = robot_.loadModel();
    setDefaultBehavior(robot_);

    command_pub_started_ = false;
    nh_.getParam("joint_states_topic", joint_states_topic_);
    nh_.getParam("joint_command_topic", joint_command_topic_);
    nh_.getParam("robot_joint_command_topic", robot_joint_command_topic_);
    nh_.getParam("joint_names", joint_names_);
    nh_.getParam("dq_max", dq_max_);

    // std::cout << joint_command_topic_ << " " << joint_states_topic_ << " " << robot_joint_command_topic_ << std::endl;
    // std::cin.ignore();

    curr_goal_state_.effort.resize(7);
    curr_goal_state_.name.resize(7);
    curr_goal_state_.position.resize(7);
    curr_goal_state_.velocity.resize(7);

    curr_robot_state_.effort.resize(7);
    curr_robot_state_.name.resize(7);
    curr_robot_state_.position.resize(7);
    curr_robot_state_.velocity.resize(7);

    curr_joint_command_.effort.resize(7);
    curr_joint_command_.name.resize(7);
    curr_joint_command_.position.resize(7);
    curr_joint_command_.velocity.resize(7);

    curr_goal_state_.name = joint_names_;
    curr_robot_state_.name = joint_names_;
    curr_joint_command_.name = joint_names_;


    // can also do tests/waits to make sure all required services, topics, etc are alive
}


// note odd syntax: &JointPositionController::subscriberCallback is a pointer to a member function of JointPositionController
// "this" keyword is required, to refer to the current instance of JointPositionController
void JointPositionController::initializeSubscribers(){
    ROS_INFO("Initializing Subscriber");
    goal_subscriber_ = nh_.subscribe("joint_command", 1, &JointPositionController::goalCallback, this);  
}

// similar syntax to subscriber, required for setting up services outside of "main()"
// void JointPositionController::initializeServices()
// {
//     ROS_INFO("Initializing Services");
//     minimal_service_ = nh_.advertiseService("exampleMinimalService",
//                                                    &JointPositionController::serviceCallback,
//                                                    this);  
//     // add more services here, as needed
// }

void JointPositionController::initializePublishers(){
    ROS_INFO("Initializing Publisher");
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, false); 
    command_publisher_ = nh_.advertise<sensor_msgs::JointState>("robot_joint_command", 1, false);
}


void JointPositionController::goalCallback(const sensor_msgs::JointState& msg) {
    curr_goal_state_ = msg;
    curr_q_des_ = Eigen::VectorXd::Map(&curr_goal_state_.position[0], 7);
    curr_dq_des_ = Eigen::VectorXd::Map(&curr_goal_state_.velocity[0], 7);
    curr_ddq_des_ = Eigen::VectorXd::Map(&curr_goal_state_.effort[0], 7);
    command_pub_started_ = true;
}


void JointPositionController::setJointPositionGoal(std::array<double, 7>& q_goal){
    curr_q_des_ = Vector7d(q_goal.data());
}

// bool JointPositionController::publishRobotState(const franka::RobotState& robot_state){
//     curr_robot_state_.header.stamp = ros::Time::now();
//     for (size_t i = 0; i < curr_robot_state_.position.size(); i++) {
//         curr_robot_state_.position[i] = robot_state.q[i];
//         curr_robot_state_.velocity[i] = robot_state.dq[i];
//         curr_robot_state_.effort[i] = robot_state.dq_d[i];
//     }
//     state_publisher_.publish(curr_robot_state_);
//     return true;
// }

bool JointPositionController::publishRobotState(const Vector7d& q, const Vector7d& dq){
    curr_robot_state_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < curr_robot_state_.position.size(); i++) {
        curr_robot_state_.position[i] = q[i];
        curr_robot_state_.velocity[i] = dq[i];
        curr_robot_state_.effort[i] = 0.0;
    }
    state_publisher_.publish(curr_robot_state_);
    return true;
}

bool JointPositionController::publishJointPosCommand(const franka::JointPositions& joint_pos_command){
    curr_joint_command_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < curr_joint_command_.position.size(); i++) {
        curr_joint_command_.position[i] = joint_pos_command.q[i];
        curr_joint_command_.velocity[i] = 0.0;
        curr_joint_command_.effort[i] = 0.0;
    }
    command_publisher_.publish(curr_joint_command_);
    return true;
}



franka::JointPositions JointPositionController::motion_generator_callback(const franka::RobotState& robot_state,
                                                                          franka::Duration period) {
    time_ += period.toSec();
    // std::array<bool, 7> joint_motion_finished{};
    bool motion_finished;

    // publishRobotState(robot_state);

    curr_q_ = Vector7d(robot_state.q_d.data());

    Vector7d q_desired;
    // q_desired = curr_q_;

    if(command_pub_started_){
        q_desired =  curr_q_des_; 
    }
    else{
        q_desired = curr_q_;
        ROS_INFO("Waiting for goal...");
    }
    // std::cout << q_desired << std::endl;
    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 7) = q_desired;
    franka::JointPositions output(joint_positions);

    if(!ros::ok()){
        ROS_INFO("Ros shutdown");
        motion_finished = true;
    }
    else{
        motion_finished = false; //std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                                    //     [](bool x) { return x; });
    }    

    output.motion_finished = motion_finished;
    publishJointPosCommand(output);
    ros::spinOnce();
    return output;

  }




franka::JointPositions JointPositionController::motion_generator_callback_integrator(const franka::RobotState& robot_state,
                                                                          franka::Duration period) {
    // int dt = period.toSec();
    double dt = 0.001;
    time_ += period.toSec();
    // std::cout << period.toSec() << std::endl;

    // double curr_time =ros::Time::now().toSec();

    // std::cout << (curr_time - prev_time_) << std::endl;
    
    // prev_time_ = curr_time;
    
    
    std::array<bool, 7> joint_motion_finished{};
    bool motion_finished;

    // publishRobotState(robot_state);

    curr_q_ = Vector7d(robot_state.q_d.data());

    if(command_pub_started_){
        delta_q_ = (curr_q_des_ - curr_q_); /// dt;

        double max_delta_q = delta_q_.lpNorm<Eigen::Infinity>();
        double dq_max_dt = dq_max_ * dt;
        // std::cout << delta_q_[0] << " " << max_delta_q <<  std::endl;

        //Normalize delta_q to respect limits

        if(max_delta_q > dq_max_dt){
            for(size_t i = 0; i < 7; i++){
                delta_q_[i] = (delta_q_[i] / max_delta_q) * dq_max_dt;
            }
        }

        // for(size_t i = 0; i < 7; i++){
        //     if(std::abs(delta_q_[i]) > dq_max_){
        //         delta_q_[i] = dq_max_;
        //     }
        // }


        // Check if goal is reached
        // std::cout << delta_q_[0] << std::endl;
        for (size_t i = 0; i < 7; i++) {
            if (std::abs(delta_q_[i]) <= kDeltaQMotionFinished) {
                // delta_q_[i] = 0.0;
                joint_motion_finished[i] = true;
            }
        } 
    }
    else{
        delta_q_.setZero();
        ROS_INFO("Waiting for goal...");
    }

    // calculate desired joint position
    Vector7d q_desired = curr_q_ + delta_q_; //(delta_q_ * dt);

    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 7) = q_desired;
    franka::JointPositions output(joint_positions);

    if(!ros::ok()){
        ROS_INFO("Ros shutdown");
        motion_finished = true;
    }
    else{
        motion_finished = false; //std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                                    //     [](bool x) { return x; });
    }    

    output.motion_finished = motion_finished;

    publishJointPosCommand(output);

    ros::spinOnce();
    return output;


  }



franka::Torques JointPositionController::torque_controller_callback(const franka::RobotState& robot_state, franka::Duration period){
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



void JointPositionController::read_loop(){
    double start_nsecs =ros::Time::now().toNSec();
    franka::Duration period;
    robot_.read([&](const franka::RobotState& robot_state) {
        
        bool ros_ok = read_state_callback(robot_state, period);
        double nsecs_elapsed = ros::Time::now().toNSec() - start_nsecs;
        
        period = franka::Duration(nsecs_elapsed/1000000.0);
        return ros_ok;
    }
    );
}

void JointPositionController::control_loop(){
    
    while(ros::ok()){
        robot_.control([&](const franka::RobotState& robot_state, franka::Duration period) 
                                            -> franka::Torques {
            return torque_controller_callback(robot_state, period);
        }
        );
    }
    std::cout << "ROS Shutdown" << std::endl;

}


bool JointPositionController::read_state_callback(const franka::RobotState& robot_state, franka::Duration period){
    // motion_generator_callback_integrator(robot_state, period);
    torque_controller_callback(robot_state, period);
    return ros::ok();

}

void JointPositionController::setDefaultBehavior(franka::Robot& robot) {
//   robot.setCollisionBehavior(
//       {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//       {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
//       {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//       {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

//   robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
//   robot.setJointImpedance({{300, 300, 300, 250, 250, 200, 200}});
//   robot.setJointImpedance({{30, 30, 30, 25, 25, 20, 20}});

//   robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}




// //member function implementation for a service callback function
// bool JointPositionController::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
//     ROS_INFO("service callback activated");
//     response.resp = true; // boring, but valid response info
//     return true;
// }




