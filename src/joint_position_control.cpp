// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include <ros/ros.h>

#include "examples_common.h"

/**
 * @example joint_position_control.cpp
 * ROS node that subscribes to a joint goal topic and moves robot to the desired
   configuration.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
//   if (argc != 10) {
//     std::cerr << "Usage: " << argv[0] << " <robot-hostname> "
//               << "<joint0> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6> "
//               << "<speed-factor>" << std::endl
//               << "joint0 to joint6 are joint angles in [rad]." << std::endl
//               << "speed-factor must be between zero and one." << std::endl;

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    double speed_factor = 0.5;

    std::array<double, 7> q_home = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}; 

    std::array<std::array<double, 7>, 2> goal_configs = {{{-M_PI_4, -M_PI_4, 0, - M_PI, 0, M_PI_2, M_PI_4}, 
                                                          {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}};

    // First move the robot to a home joint configuration
    MotionGenerator motion_generator(speed_factor, q_home);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;


    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    std::array<double, 7> q_goal;

    for(size_t i = 0; i < goal_configs.size(); ++i){
        q_goal = goal_configs[i];

        motion_generator.UpdateGoal(q_goal);
        std::cout << "WARNING: This example will move the robot! "
                    << "Please make sure to have the user stop button at hand!" << std::endl
                    << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Motion finished" << std::endl;

    }
    
    // for (size_t i = 0; i < 7; i++) {
    //   q_goal[i] = std::stod(argv[i + 2]);
    // }
    // double speed_factor = std::stod(argv[9]);

    // Finally move back to home_config
    motion_generator.UpdateGoal(q_home);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
