#pragma once

// #include <math.h>
// #include <stdlib.h>
// #include <string>
// #include <vector>

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>

#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


// #include <std_msgs/Bool.h> 
// #include <std_msgs/Float32.h>


// #include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package

class JointPositionController
{
public:
    JointPositionController(ros::NodeHandle* nodehandle, double dq_max); //std::string robot_ip
    void setDefaultBehavior(franka::Robot& robot);
    // franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);
    franka::JointPositions motion_generator_callback(const franka::RobotState& robot_state, franka::Duration period);
    franka::JointPositions motion_generator_callback_integrator(const franka::RobotState& robot_state, franka::Duration period);

    bool read_state_callback(const franka::RobotState& robot_state);
    void setJointPositionGoal(std::array<double, 7>& q_goal);

private:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    std::string joint_states_topic_;
    std::string joint_command_topic_;
    std::string robot_joint_command_topic_;
    ros::Subscriber goal_subscriber_;     
    // ros::ServiceServer minimal_service_;
    ros::Publisher  state_publisher_;
    ros::Publisher  command_publisher_;

    std::vector<std::string> joint_names_;
    sensor_msgs::JointState curr_robot_state_;
    sensor_msgs::JointState curr_goal_state_;
    sensor_msgs::JointState curr_joint_command_;
    // franka::RobotState curr_robot_state_;

    Vector7d curr_q_goal_;
    Vector7d curr_q_;
    Vector7d delta_q_;
    double time_ = 0.0;
    double dq_max_;
    bool goal_pub_started_;

    static constexpr double kDeltaQMotionFinished = 1e-6;



    // Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
    // Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
    // Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    // void initializeServices();
    void goalCallback(const sensor_msgs::JointState& msg);
    bool publishRobotState(const franka::RobotState& robot_state);
    bool publishJointPosCommand(const franka::JointPositions& joint_pos_command);


         //prototype for callback for example service
    // bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
};

