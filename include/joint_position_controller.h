#pragma once

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
#include "franka_controller.h"


class JointPositionController : public FrankaController
{
public:
    JointPositionController(ros::NodeHandle* nh, ros::NodeHandle* pnh, std::string robot_ip); 
    void command_loop();
    void initialize_control_gains();


private:
    franka::JointPositions joint_position_controller_callback(const franka::RobotState& robot_state, franka::Duration period);
    double alpha_q_, alpha_dq_;
    bool gains_set_;   
};

