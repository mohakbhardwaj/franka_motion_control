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





class JointPositionController
{
public:
    // JointPositionController(ros::NodeHandle* nodehandle, franka::Robot& robot); 
    JointPositionController(ros::NodeHandle* nodehandle, std::string robot_ip); 
    void setDefaultBehavior(franka::Robot& robot);
    // franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);
    franka::JointPositions motion_generator_callback(const franka::RobotState& robot_state, franka::Duration period);
    franka::JointPositions motion_generator_callback_integrator(const franka::RobotState& robot_state, franka::Duration period);
    franka::Torques torque_controller_callback(const franka::RobotState& robot_state, franka::Duration period);
    bool read_state_callback(const franka::RobotState& robot_state, franka::Duration period);
    void setJointPositionGoal(std::array<double, 7>& q_goal);
    void read_loop();
    void control_loop();

private:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    using Mat7d = Eigen::Matrix<double, 7, 7, Eigen::ColMajor>;
    using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

    using Vector7f = Eigen::Matrix<float, 7, 1, Eigen::ColMajor>;
    using Mat7f = Eigen::Matrix<float, 7, 7, Eigen::ColMajor>;
    
    std::string robot_ip_;
    franka::Robot robot_;
    franka::Model model_;

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

    Vector7d curr_q_des_;
    Vector7d curr_dq_des_;
    Vector7d curr_ddq_des_;
    Vector7d curr_q_;
    Vector7d curr_dq_;
    Vector7d delta_q_;

    Vector7d curr_q_bel_, curr_dq_bel_, prev_q_bel_; //filtered belief over state

    Vector7d q_des_cmd_, dq_des_cmd_, ddq_des_cmd_;


    double time_ = 0.0;
    double dq_max_;
    bool command_pub_started_;

    double prev_time_ = 0.0;

    static constexpr double kDeltaQMotionFinished = 1e-6;



    // Vector7f dq_max_ = (Vector7f() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
    // Vector7f ddq_max_start_ = (Vector7f() << 5, 5, 5, 5, 5, 5, 5).finished();
    // Vector7f ddq_max_goal_ = (Vector7f() << 5, 5, 5, 5, 5, 5, 5).finished();
   
    // Vector7f P_ = (Vector7f() << 6.0, 6.0, 6.0, 6.0, 2.5, 4.0, 4.0).finished();

    // Vector7f P_ = (Vector7f() << 7.0, 5.0, 5.0, 7.0, 5.0, 6.0, 7.0).finished();
    // Vector7f D_ = (Vector7f() << 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.5).finished();
    double alpha_q_ = 0.8; //1.0;
    double alpha_dq_ = 0.01; //0.05; //1.0; 
    Vector7d Pf_ = (Vector7d() << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5).finished();
    

    // Vector7d P_ = (Vector7d() << 210.0, 200.0, 5.0, 7.0, 5.0, 6.0, 7.0).finished();
    // Vector7d P_ = (Vector7d() << 100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0).finished();    
    // Vector7d D_ = (Vector7d() << 12.0, 12.0, 12.0, 12.0, 2.0, 2.0, 1.0).finished();
    
    //star
    // Vector7d P_ = (Vector7d() << 90.0, 90.0, 90.0, 90.0, 50.0, 50.0, 50.0).finished();    
    // Vector7d D_ = (Vector7d() << 16.0, 16.0, 16.0, 16.0, 3.0, 3.0, 3.0).finished();

    // Vector7d P_ = (Vector7d() << 80.0, 80.0, 80.0, 80.0, 50.0, 50.0, 50.0).finished();    
    // Vector7d D_ = (Vector7d() << 16.0, 16.0, 16.0, 16.0, 3.0, 3.0, 3.0).finished();


    // Vector7d P_ = (Vector7d() << 70.0, 70.0, 70.0, 70.0, 40.0, 40.0, 40.0).finished();    
    // Vector7d D_ = (Vector7d() << 16.0, 16.0, 16.0, 16.0, 3.0, 3.0, 3.0).finished();


    // Vector7d P_ = (Vector7d() << 300.0, 300.0, 300.0, 300.0, 100.0, 100.0, 100.0).finished();    
    // Vector7d D_ = (Vector7d() << 20.0, 20.0, 20.0, 25.0, 2.0, 2.0, 2.0).finished();

    Vector7d P_ = (Vector7d() << 650.0, 650.0, 650.0, 650.0, 200.0, 100.0, 50.0).finished();    
    // Vector7d D_ = (Vector7d() << 25.0, 25.0, 25.0, 25.0, 2.0, 2.0, 2.0).finished();
    Vector7d D_ = (Vector7d() << 30.0, 30.0, 30.0, 30.0, 5.0, 5.0, 2.0).finished();


    // Vector7d P_ = (Vector7d() << 600.0, 600.0, 600.0, 600.0, 250.0, 100.0, 50.0).finished();    
    // Vector7d D_ = (Vector7d() << 50.0, 50.0, 50.0, 50.0, 25.0, 25.0, 15.0).finished();
    // Vector7d D_ = (Vector7d() << 25.0, 25.0, 25.0, 25.0, 2.0, 2.0, 2.0).finished();
    // Vector7d D_ = (Vector7d() << 20.0, 20.0, 20.0, 20.0, 2.0, 2.0, 2.0).finished();
    // Vector7d D_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 0.2, 0.2, 2.0).finished();

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    // void initializeServices();
    void goalCallback(const sensor_msgs::JointState& msg);
    // bool publishRobotState(const franka::RobotState& robot_state);
    bool publishRobotState(const Vector7d& q, const Vector7d& dq);

    bool publishJointPosCommand(const franka::JointPositions& joint_pos_command);

         //prototype for callback for example service
    // bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
};

