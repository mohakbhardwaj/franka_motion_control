#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <functional>

#include <franka/exception.h>
#include <franka/robot.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "joint_position_controller.h"



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "JointPositionController"); //node name

    ros::NodeHandle nh("~");

    std::vector<std::string> joint_names;
    nh.getParam("joint_names", joint_names);

    std::string robot_ip; // ="172.16.0.2";
    nh.getParam("robot_ip", robot_ip);
    ROS_INFO("robot_ip: %s", robot_ip.c_str());
    
    double dq_max;
    nh.getParam("dq_max", dq_max);
    ROS_INFO("dq_max %f", dq_max);

    bool debug;
    nh.getParam("debug", debug);
    ROS_INFO("Debug mode: %d",  debug);

    ROS_INFO("main: instantiating JointPositionController");
    JointPositionController controller(&nh, dq_max);  //instantiate an JointPositionController object and pass in pointer to nodehandle for constructor to use

    try {
        ROS_INFO("Connecting to robot %s", robot_ip);
        franka::Robot robot(robot_ip);
        controller.setDefaultBehavior(robot);
        // First move the robot to a home joint configuration
        // std::array<double, 7> q_home ={{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        // controller.setJointPositionGoal(q_home);   
     
        if(debug){
            std::cout << "Here" << std::endl;
            // reads robot state but does not execute commands
            robot.read([&](const franka::RobotState& robot_state) {
                return controller.read_state_callback(robot_state);               
            }
            );
        }
        else{
            //actually controls the robot using published commands
            ROS_INFO("WARNING: This example will move the robot! \n"
                        "Please make sure to have the user stop button at hand! \n"
                        "Press Enter to continue... \n");
            std::cin.ignore();            
            
            robot.control([&](const franka::RobotState& robot_state, franka::Duration period) 
                                       -> franka::JointPositions {
                return controller.motion_generator_callback(robot_state, period);
            }
        );
        }
        ROS_INFO("Finished motion.");


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}















// #include <algorithm>
// #include <cmath>
// #include <iostream>
// #include <vector>

// #include <franka/exception.h>
// #include <franka/robot.h>

// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>

// #include "examples_common.h"



// void goalCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }


// int main(int argc, char** argv) {
//   // if (argc != 2) {
//   //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//   //   return -1;
//   // }
//   ros::init(argc, argv, "joint_position_control");
//   ros::NodeHandle nh;

//   std::vector<std::string> joint_names;
//   node_handle.getParam("joint_names", joint_names);

//   std::string robot_ip;
//   node_handle.getParam("robot_ip", robot_ip);

//   ros::Subscriber sub = n.subscribe("joint_pos_goal", 1000, goalCallback);

//   ros::AsyncSpinner spinner(1);

//   spinner.start();



//   try {
//     franka::Robot robot(robot_ip);
//     setDefaultBehavior(robot);
//     double speed_factor = 0.5;

//     // First move the robot to a home joint configuration
    
    
//     MotionGenerator motion_generator(speed_factor, q_home);
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(motion_generator);
//     std::cout << "Finished moving to initial joint configuration." << std::endl;


//     // Set additional parameters always before the control loop, NEVER in the control loop!
//     // Set collision behavior.
//     robot.setCollisionBehavior(
//         {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//         {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
//         {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//         {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

//     // std::array<double, 7> q_goal;

//     for(size_t i = 0; i < goal_configs.size(); ++i){
//         q_goal = goal_configs[i];

//         motion_generator.UpdateGoal(q_goal);
//         std::cout << "WARNING: This example will move the robot! "
//                     << "Please make sure to have the user stop button at hand!" << std::endl
//                     << "Press Enter to continue..." << std::endl;
//         std::cin.ignore();
//         robot.control(motion_generator);
//         std::cout << "Motion finished" << std::endl;

//     }

//     // Finally move back to home_config
//     motion_generator.UpdateGoal(q_home);
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(motion_generator);
//     std::cout << "Finished moving to initial joint configuration." << std::endl;

//   } catch (const franka::Exception& e) {
//     std::cout << e.what() << std::endl;
//     return -1;
//   }

//   return 0;
// }
