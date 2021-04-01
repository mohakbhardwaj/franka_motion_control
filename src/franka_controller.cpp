#include "franka_controller.h"

FrankaController::FrankaController(ros::NodeHandle* nh, ros::NodeHandle* pnh, std::string robot_ip):
                        nh_(*nh), pnh_(*pnh), robot_(robot_ip), model_(robot_.loadModel()){   
    

    curr_q_.setZero();
    curr_dq_.setZero();
    curr_q_des_.setZero();
    curr_dq_des_.setZero();
    curr_ddq_des_.setZero();
    curr_q_bel_.setZero();
    curr_dq_bel_.setZero();
    tau_z_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    
    setDefaultBehavior(robot_);

    command_pub_started_ = false;
    nh_.getParam("joint_states_topic", joint_states_topic_);
    nh_.getParam("joint_command_topic", joint_command_topic_);
    nh_.getParam("robot_joint_command_topic", robot_joint_command_topic_);
    nh_.getParam("prefix", prefix_);
    pnh_.getParam("joint_names", joint_names_);

    joint_states_topic_ = prefix_ + "/" + joint_states_topic_;
    joint_command_topic_ = prefix_ + "/" + joint_command_topic_;
    robot_joint_command_topic_ = prefix_ + "/" + robot_joint_command_topic_;

    initializeSubscribers(); 
    initializePublishers();

    curr_joint_command_.name.resize(7);
    curr_joint_command_.position.resize(7);
    curr_joint_command_.velocity.resize(7);
    curr_joint_command_.effort.resize(7);

    curr_robot_state_.name.resize(7);
    curr_robot_state_.position.resize(7);
    curr_robot_state_.velocity.resize(7);
    curr_robot_state_.effort.resize(7);

    curr_robot_joint_command_.name.resize(7);
    curr_robot_joint_command_.position.resize(7);
    curr_robot_joint_command_.velocity.resize(7);
    curr_robot_joint_command_.effort.resize(7);

    curr_joint_command_.name = joint_names_;
    curr_robot_state_.name = joint_names_;
    curr_robot_joint_command_.name = joint_names_;
}

void FrankaController::initializeSubscribers(){
    ROS_INFO("Initializing Subscriber");
    goal_subscriber_ = nh_.subscribe(joint_command_topic_, 1, &FrankaController::jointCommandCallback, this);  
}

void FrankaController::initializePublishers(){
    ROS_INFO("Initializing Publisher");
    state_publisher_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 1, false); 
    command_publisher_ = nh_.advertise<sensor_msgs::JointState>(robot_joint_command_topic_, 1, false);
}

void FrankaController::jointCommandCallback(const sensor_msgs::JointState& msg) {
    curr_joint_command_ = msg;
    curr_q_des_ = Eigen::VectorXd::Map(&curr_joint_command_.position[0], 7);
    curr_dq_des_ = Eigen::VectorXd::Map(&curr_joint_command_.velocity[0], 7);
    curr_ddq_des_ = Eigen::VectorXd::Map(&curr_joint_command_.effort[0], 7);
    command_pub_started_ = true;
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

bool FrankaController::publishRobotCommand(const franka::RobotState& robot_command){
    curr_robot_joint_command_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < curr_robot_joint_command_.position.size(); i++) {
        curr_robot_joint_command_.position[i] = robot_command.q[i];
        curr_robot_joint_command_.velocity[i] = robot_command.dq[i];
        curr_robot_joint_command_.effort[i] = robot_command.tau_J[i];
    }
    command_publisher_.publish(curr_robot_joint_command_);
    return true;
}

void FrankaController::monitor_loop(){
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

void FrankaController::gravity_command_loop(){
    
    while(ros::ok()){
        robot_.control([&](const franka::RobotState& robot_state, franka::Duration period) 
                                            -> franka::Torques {            
            publishRobotState(robot_state);
            return tau_z_;
        }
        );
    }

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




