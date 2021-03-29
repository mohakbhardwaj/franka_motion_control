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
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>





class FrankaController
{
public:
    FrankaController(ros::NodeHandle* nodehandle, std::string robot_ip); 
    void setDefaultBehavior(franka::Robot& robot);
    // franka::JointPositions motion_generator_callback(const franka::RobotState& robot_state, franka::Duration period);
    // franka::JointPositions motion_generator_callback_integrator(const franka::RobotState& robot_state, franka::Duration period);
    // franka::Torques torque_controller_callback(const franka::RobotState& robot_state, franka::Duration period);
    // bool read_state_callback(const franka::RobotState& robot_state, franka::Duration period);
    void setJointPositionGoal(std::array<double, 7>& q_goal);
    void read_loop();
    virtual void control_loop()=0;


protected:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    using Mat7d = Eigen::Matrix<double, 7, 7, Eigen::ColMajor>;
    using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;
    using Vector7f = Eigen::Matrix<float, 7, 1, Eigen::ColMajor>;
    using Mat7f = Eigen::Matrix<float, 7, 7, Eigen::ColMajor>;
    
    std::string robot_ip_;
    franka::Robot robot_;
    franka::Model model_;

    ros::NodeHandle nh_; 
    std::string joint_states_topic_;
    std::string joint_command_topic_;
    std::string robot_joint_command_topic_;
    ros::Subscriber goal_subscriber_;     
    ros::Publisher  state_publisher_;
    ros::Publisher  command_publisher_;

    std::vector<std::string> joint_names_;
    sensor_msgs::JointState curr_robot_state_;
    sensor_msgs::JointState curr_goal_state_;
    sensor_msgs::JointState curr_joint_command_;
    // franka::RobotState curr_robot_state_;

    Vector7d curr_q_des_;
    Vector7d curr_dq_des_;
    Vector7d curr_ddq_des_;
    Vector7d curr_q_;
    Vector7d curr_dq_;
    Vector7d delta_q_;
    Vector7d curr_q_bel_, curr_dq_bel_, prev_q_bel_; //filtered belief over state
    Vector7d q_des_cmd_, dq_des_cmd_, ddq_des_cmd_;

    double time_ = 0.0;
    // double dq_max_;
    bool command_pub_started_;

    double prev_time_ = 0.0;

    static constexpr double kDeltaQMotionFinished = 1e-6;



    void initializeSubscribers();
    void initializePublishers();
    void jointCommandCallback(const sensor_msgs::JointState& msg);
    bool publishRobotState(const franka::RobotState& robot_state);
    bool publishRobotState(const Vector7d& q, const Vector7d& dq);
    bool publishJointPosCommand(const franka::JointPositions& joint_pos_command);

};

