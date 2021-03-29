#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <functional>

#include <franka/exception.h>
#include <franka/robot.h>


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "torque_controller.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "FrankaTorqueController");

    ros::NodeHandle nh("~");

    std::string robot_ip;
    nh.getParam("robot_ip", robot_ip);
    ROS_INFO("robot_ip: %s", robot_ip.c_str());
    
    bool debug;
    nh.getParam("debug", debug);
    ROS_INFO("Debug mode: %d",  debug);

    try {
        TorqueController controller(&nh, robot_ip);

        if(debug){
            //read robot state and publish
            controller.read_loop();
        }
        else{
            //control the robot using published joint commands
            ROS_INFO("WARNING: This example will move the robot! \n"
                        "Please make sure to have the user stop button at hand! \n"
                        "Press Enter to continue... \n");
            std::cin.ignore();            
            controller.control_loop();        
        }
        ROS_INFO("Finished motion.");


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}