#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "examples_common.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "move_to_home");
  ros::NodeHandle nh("~");

  std::vector<std::string> joint_names;
  nh.getParam("joint_names", joint_names);

  std::string robot_ip;
  nh.getParam("robot_ip", robot_ip);
  ROS_INFO("robot_ip: %s", robot_ip.c_str());


  std::array<double, 7> q_home = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}; 


  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    double speed_factor = 0.5;

    
    MotionGenerator motion_generator(speed_factor, q_home);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration. Exiting..." << std::endl;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
