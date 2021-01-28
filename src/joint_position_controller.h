#ifndef JOINT_POSITION_CONTROLLER_
#define JOINT_POSITION_CONTROLLER_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// #include <std_msgs/Bool.h> 
// #include <std_msgs/Float32.h>


// #include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package

// define a class, including a constructor, member variables and member functions
class JointPositionController
{
public:
    JointPositionController(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    ros::Subscriber goal_subscriber_;     
    // ros::ServiceServer minimal_service_;
    ros::Publisher  state_publisher_;
    
    // double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    // double val_to_remember_; // member variables will retain their values even as callbacks come and go
    
    // member methods as well:
    // void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    // void initializePublishers();
    // void initializeServices();
    
    void goalCallback(const sensor_msgs::JointState& msg); 
    //prototype for callback for example service
    // bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
