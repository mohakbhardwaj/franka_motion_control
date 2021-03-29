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
  if (argc != 10) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
              << "<joint0> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6> "
              << "<speed-factor>" << std::endl
              << "joint0 to joint6 are joint angles in [rad]." << std::endl
              << "speed-factor must be between zero and one." << std::endl;
    return -1;


  // ros::init(argc, argv, "move_to_home");
  // ros::NodeHandle nh("~");

  // std::vector<std::string> joint_names;
  // nh.getParam("joint_names", joint_names);

  // std::string robot_ip;
  // nh.getParam("robot_ip", robot_ip);
  // ROS_INFO("robot_ip: %s", robot_ip.c_str());


  // std::array<double, 7> q_home = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}; 

  // std::array<double, 7> q_home = {{0.0, -0.7853, 0.0, -2.356, 0.0, 3.14, -0.785}};

  // std::array<double, 7> q_home = {{-0.207, -0.494, -0.398, -2.239, 1.289, 1.194, 0.462}};
  // std::array<double, 7> q_home = {{1.745, 0.914, -2.254, -2.306, 0.784, 1.617, -0.109}}; 
  // std::array<double, 7> q_home = {{-0.9, -0.9, 1.2, -1.9, -0.2, 1.57, 0.78}}; 

  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);
    std::array<double, 7> q_goal;
    for (size_t i = 0; i < 7; i++) {
      q_goal[i] = std::stod(argv[i + 2]);
    }
    double speed_factor = std::stod(argv[9]);
    // double speed_factor = 0.3;

    
    MotionGenerator motion_generator(speed_factor, q_goal);
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
