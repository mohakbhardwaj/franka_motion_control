#include "franka_controller.h"

FrankaController::FrankaController(ros::NodeHandle* nodehandle, std::string robot_ip):
                        nh_(*nodehandle), robot_(robot_ip), model_(robot_.loadModel()){   
    
    // ROS_INFO("in class constructor of FrankaController");
    initializeSubscribers(); 
    initializePublishers();
    curr_q_.setZero();
    curr_dq_.setZero();
    curr_q_des_.setZero();
    curr_dq_des_.setZero();
    curr_ddq_des_.setZero();
    delta_q_.setZero();
    curr_q_bel_.setZero();
    curr_dq_bel_.setZero();
    
    setDefaultBehavior(robot_);

    command_pub_started_ = false;
    nh_.getParam("joint_states_topic", joint_states_topic_);
    nh_.getParam("joint_command_topic", joint_command_topic_);
    nh_.getParam("robot_joint_command_topic", robot_joint_command_topic_);
    nh_.getParam("joint_names", joint_names_);


    curr_goal_state_.name.resize(7);
    curr_goal_state_.position.resize(7);
    curr_goal_state_.velocity.resize(7);
    curr_goal_state_.effort.resize(7);

    curr_robot_state_.name.resize(7);
    curr_robot_state_.position.resize(7);
    curr_robot_state_.velocity.resize(7);
    curr_robot_state_.effort.resize(7);

    curr_joint_command_.name.resize(7);
    curr_joint_command_.position.resize(7);
    curr_joint_command_.velocity.resize(7);
    curr_joint_command_.effort.resize(7);

    curr_goal_state_.name = joint_names_;
    curr_robot_state_.name = joint_names_;
    curr_joint_command_.name = joint_names_;
}

void FrankaController::initializeSubscribers(){
    ROS_INFO("Initializing Subscriber");
    goal_subscriber_ = nh_.subscribe("joint_command", 1, &FrankaController::jointCommandCallback, this);  
}

void FrankaController::initializePublishers(){
    ROS_INFO("Initializing Publisher");
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, false); 
    command_publisher_ = nh_.advertise<sensor_msgs::JointState>("robot_joint_command", 1, false);
}

void FrankaController::jointCommandCallback(const sensor_msgs::JointState& msg) {
    curr_goal_state_ = msg;
    curr_q_des_ = Eigen::VectorXd::Map(&curr_goal_state_.position[0], 7);
    curr_dq_des_ = Eigen::VectorXd::Map(&curr_goal_state_.velocity[0], 7);
    curr_ddq_des_ = Eigen::VectorXd::Map(&curr_goal_state_.effort[0], 7);
    command_pub_started_ = true;
}

void FrankaController::setJointPositionGoal(std::array<double, 7>& q_goal){
    curr_q_des_ = Vector7d(q_goal.data());
}

bool FrankaController::publishRobotState(const franka::RobotState& robot_state){
    curr_robot_state_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < curr_robot_state_.position.size(); i++) {
        curr_robot_state_.position[i] = robot_state.q[i];
        curr_robot_state_.velocity[i] = robot_state.dq[i];
        curr_robot_state_.effort[i] = 0.0; //robot_state.dq_d[i];
    }
    state_publisher_.publish(curr_robot_state_);
    return true;
}

bool FrankaController::publishRobotState(const Vector7d& q, const Vector7d& dq){
    curr_robot_state_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < curr_robot_state_.position.size(); i++) {
        curr_robot_state_.position[i] = q[i];
        curr_robot_state_.velocity[i] = dq[i];
        curr_robot_state_.effort[i] = 0.0;
    }
    state_publisher_.publish(curr_robot_state_);
    return true;
}

bool FrankaController::publishJointPosCommand(const franka::JointPositions& joint_pos_command){
    curr_joint_command_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < curr_joint_command_.position.size(); i++) {
        curr_joint_command_.position[i] = joint_pos_command.q[i];
        curr_joint_command_.velocity[i] = 0.0;
        curr_joint_command_.effort[i] = 0.0;
    }
    command_publisher_.publish(curr_joint_command_);
    return true;
}


// franka::JointPositions FrankaController::motion_generator_callback(const franka::RobotState& robot_state,
//                                                                           franka::Duration period) {
//     time_ += period.toSec();
//     // std::array<bool, 7> joint_motion_finished{};
//     bool motion_finished;

//     // publishRobotState(robot_state);

//     curr_q_ = Vector7d(robot_state.q_d.data());

//     Vector7d q_desired;
//     // q_desired = curr_q_;

//     if(command_pub_started_){
//         q_desired =  curr_q_des_; 
//     }
//     else{
//         q_desired = curr_q_;
//         ROS_INFO("Waiting for goal...");
//     }
//     // std::cout << q_desired << std::endl;
//     std::array<double, 7> joint_positions;
//     Eigen::VectorXd::Map(&joint_positions[0], 7) = q_desired;
//     franka::JointPositions output(joint_positions);

//     if(!ros::ok()){
//         ROS_INFO("Ros shutdown");
//         motion_finished = true;
//     }
//     else{
//         motion_finished = false; //std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
//                                     //     [](bool x) { return x; });
//     }    

//     output.motion_finished = motion_finished;
//     publishJointPosCommand(output);
//     ros::spinOnce();
//     return output;

//   }




// franka::JointPositions FrankaController::motion_generator_callback_integrator(const franka::RobotState& robot_state,
                                                                        //   franka::Duration period) {
    // int dt = period.toSec();
//     double dt = 0.001;
//     time_ += period.toSec();
//     // std::cout << period.toSec() << std::endl;

//     // double curr_time =ros::Time::now().toSec();

//     // std::cout << (curr_time - prev_time_) << std::endl;
    
//     // prev_time_ = curr_time;
    
    
//     std::array<bool, 7> joint_motion_finished{};
//     bool motion_finished;

//     // publishRobotState(robot_state);

//     curr_q_ = Vector7d(robot_state.q_d.data());

//     if(command_pub_started_){
//         delta_q_ = (curr_q_des_ - curr_q_); /// dt;

//         double max_delta_q = delta_q_.lpNorm<Eigen::Infinity>();
//         double dq_max_dt = dq_max_ * dt;
//         // std::cout << delta_q_[0] << " " << max_delta_q <<  std::endl;

//         //Normalize delta_q to respect limits

//         if(max_delta_q > dq_max_dt){
//             for(size_t i = 0; i < 7; i++){
//                 delta_q_[i] = (delta_q_[i] / max_delta_q) * dq_max_dt;
//             }
//         }

//         // for(size_t i = 0; i < 7; i++){
//         //     if(std::abs(delta_q_[i]) > dq_max_){
//         //         delta_q_[i] = dq_max_;
//         //     }
//         // }


//         // Check if goal is reached
//         // std::cout << delta_q_[0] << std::endl;
//         for (size_t i = 0; i < 7; i++) {
//             if (std::abs(delta_q_[i]) <= kDeltaQMotionFinished) {
//                 // delta_q_[i] = 0.0;
//                 joint_motion_finished[i] = true;
//             }
//         } 
//     }
//     else{
//         delta_q_.setZero();
//         ROS_INFO("Waiting for goal...");
//     }

//     // calculate desired joint position
//     Vector7d q_desired = curr_q_ + delta_q_; //(delta_q_ * dt);

//     std::array<double, 7> joint_positions;
//     Eigen::VectorXd::Map(&joint_positions[0], 7) = q_desired;
//     franka::JointPositions output(joint_positions);

//     if(!ros::ok()){
//         ROS_INFO("Ros shutdown");
//         motion_finished = true;
//     }
//     else{
//         motion_finished = false; //std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
//                                     //     [](bool x) { return x; });
//     }    

//     output.motion_finished = motion_finished;

//     publishJointPosCommand(output);

//     ros::spinOnce();
//     return output;


//   }


void FrankaController::read_loop(){
    // double start_nsecs =ros::Time::now().toNSec();
    // franka::Duration period;
    
    robot_.read([&](const franka::RobotState& robot_state) {
        

        // bool ros_ok = read_state_callback(robot_state, period);
        // double nsecs_elapsed = ros::Time::now().toNSec() - start_nsecs;
        
        // period = franka::Duration(nsecs_elapsed/1000000.0);
        publishRobotState(robot_state);
        return ros::ok();
    }
    );
}



// bool FrankaController::read_state_callback(const franka::RobotState& robot_state, franka::Duration period){
//     // motion_generator_callback_integrator(robot_state, period);
//     torque_controller_callback(robot_state, period);
//     return ros::ok();

// }


void FrankaController::setDefaultBehavior(franka::Robot& robot) {
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




