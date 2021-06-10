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
    // void initialize_control_gains();


private:
    franka::JointPositions joint_position_controller_callback(const franka::RobotState& robot_state, franka::Duration period);

    // Vector7d tau_d_error_, tau_d_coriolis_, tau_d_inertia_, tau_d_calculated_;
    // std::array<double, 7> tau_d_calculated_arr_;

    double alpha_q_, alpha_dq_;
    Vector7d Km_, Kp_, Kd_;
    // bool gains_set_;   
};

