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
  ros::init(argc, argv, "move_to_joint_config");
  ros::NodeHandle nh("~");

  std::string robot_ip;
  nh.getParam("robot_ip", robot_ip);
  std::vector<double> goal_config;
  nh.getParam("goal_config", goal_config);
  double speed_factor;
  nh.getParam("speed_factor", speed_factor);

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    std::array<double,7> goal_config_arr;
    std::copy(goal_config.begin(), goal_config.begin()+7, goal_config_arr.begin());
    
    MotionGenerator motion_generator(speed_factor, goal_config_arr);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to joint configuration. Exiting..." << std::endl;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
