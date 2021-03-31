#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <functional>
#include <stdexcept>

#include <franka/exception.h>
#include <franka/robot.h>


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "tracking_controller.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "FrankaTorqueController");

    ros::NodeHandle nh("~");

    std::string robot_ip;
    nh.getParam("robot_ip", robot_ip);
    ROS_INFO("robot_ip: %s", robot_ip.c_str());
    
    // bool monitor;
    // nh.getParam("monitor", monitor);
    // ROS_INFO("monitor mode: %d",  monitor);

    std::string mode;
    nh.getParam("mode", mode);
    ROS_INFO('Mode: %d', mode)

    try {
        TorqueController controller(&nh, robot_ip);

        if (mode == "monitor"){
            //read robot state and publish
            controller.monitor_loop();
        }
        else if (mode == "gravity"){
            //puts arm in gravity compensation mode
            controller.gravity_command_loop();
        }
        else if (mode == "command"){
            //control the robot using published joint commands
            ROS_INFO("WARNING: This example will move the robot! \n"
                        "Please make sure to have the user stop button at hand! \n"
                        "Press Enter to continue... \n");
            std::cin.ignore();            
            controller.command_loop();        
        }
        else{
            throw std::runtime_error("Invalid operation mode");
        }


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}