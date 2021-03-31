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


class TrackingController : public FrankaController
{
public:
    TrackingController(ros::NodeHandle* nh, ros::NodeHandle* pnh, std::string robot_ip); 
    void command_loop();
    void initialize_control_gains();


private:
    franka::Torques torque_controller_callback(const franka::RobotState& robot_state, franka::Duration period);

    Vector7d tau_d_error_, tau_d_coriolis_, tau_d_inertia_, tau_d_calculated_;
    std::array<double, 7> tau_d_calculated_arr_;

    // Vector7f dq_max_ = (Vector7f() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
    // Vector7f ddq_max_start_ = (Vector7f() << 5, 5, 5, 5, 5, 5, 5).finished();
    // Vector7f ddq_max_goal_ = (Vector7f() << 5, 5, 5, 5, 5, 5, 5).finished();
   
    // Vector7f P_ = (Vector7f() << 6.0, 6.0, 6.0, 6.0, 2.5, 4.0, 4.0).finished();

    // Vector7f P_ = (Vector7f() << 7.0, 5.0, 5.0, 7.0, 5.0, 6.0, 7.0).finished();
    // Vector7f D_ = (Vector7f() << 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.5).finished();
    double alpha_q_, alpha_dq_;
    Vector7d Km_, Kp_, Kd_;
    bool gains_set_;   
    ////Use these
    // double alpha_q_ = 0.8; //1.0;
    // double alpha_dq_ = 0.01; //0.05; //1.0; 
    // Vector7d Pf_ = (Vector7d() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished();
    ////

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

    // Vector7d P_ = (Vector7d() << 500.0, 500.0, 500.0, 500.0, 100.0, 100.0, 50.0).finished();    
    // Vector7d P_ = (Vector7d() << 300.0, 300.0, 300.0, 300.0, 100.0, 100.0, 50.0).finished();
    // Vector7d D_ = (Vector7d() << 20.0, 20.0, 20.0, 25.0, 2.0, 2.0, 2.0).finished();
    
    // Vector7d D_ = (Vector7d() << 25.0, 25.0, 25.0, 25.0, 2.0, 2.0, 2.0).finished();
    // Vector7d D_ = (Vector7d() << 30.0, 30.0, 30.0, 30.0, 2.0, 2.0, 2.0).finished();


    // Vector7d P_ = (Vector7d() << 600.0, 600.0, 600.0, 600.0, 100.0, 100.0, 50.0).finished();    
    // // Vector7d D_ = (Vector7d() << 50.0, 50.0, 50.0, 50.0, 5.0, 5.0, 5.0).finished();
    // Vector7d D_ = (Vector7d() << 25.0, 25.0, 25.0, 25.0, 2.0, 2.0, 2.0).finished();
    // // Vector7d D_ = (Vector7d() << 20.0, 20.0, 20.0, 20.0, 2.0, 2.0, 2.0).finished();
    // Vector7d D_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 0.2, 0.2, 2.0).finished();


    // Vector7d P_ = (Vector7d() << 650.0, 650.0, 650.0, 650.0, 200.0, 100.0, 50.0).finished();   
    ///Use this 
    // Vector7d P_ = (Vector7d() << 650.0, 650.0, 650.0, 650.0, 100.0, 100.0, 50.0).finished();    
    //////
    // Vector7d D_ = (Vector7d() << 25.0, 25.0, 25.0, 25.0, 2.0, 2.0, 2.0).finished();
    
    //Use this
    // Vector7d D_ = (Vector7d() << 30.0, 30.0, 30.0, 30.0, 5.0, 5.0, 2.0).finished();
    /////
};

