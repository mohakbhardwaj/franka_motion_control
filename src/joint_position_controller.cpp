// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun example_ros_class example_ros_class
// rostopic echo exampleMinimalPubTopic
// rostopic pub -r 4 exampleMinimalSubTopic std_msgs/Float32 2.0
// rosservice call exampleMinimalService 1


// this header incorporates all the necessary #include files and defines the class "JointPositionController"
#include "joint_position_controller.h"
// #include<ros/ros.h>

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
JointPositionController::JointPositionController(ros::NodeHandle* nodehandle, double dq_max):
                        nh_(*nodehandle), dq_max_(dq_max) //std::string robot_ip, robot_(robot_ip), 
{ // constructor
    ROS_INFO("in class constructor of JointPositionController");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    // initializePublishers();
    // initializeServices();
    // robot_ip_ = robot_ip;
    // setDefaultBehavior(robot_);
    // initialize variables here

    curr_q_.setZero();
    curr_q_goal_.setZero();
    delta_q_.setZero();
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}


// note odd syntax: &JointPositionController::subscriberCallback is a pointer to a member function of JointPositionController
// "this" keyword is required, to refer to the current instance of JointPositionController
void JointPositionController::initializeSubscribers(){
    // ROS_INFO("Initializing Subscribers");
    goal_subscriber_ = nh_.subscribe("joint_position_goal", 1, &JointPositionController::goalCallback,this);  
    // add more subscribers here, as needed
}

// similar syntax to subscriber, required for setting up services outside of "main()"
// void JointPositionController::initializeServices()
// {
//     ROS_INFO("Initializing Services");
//     minimal_service_ = nh_.advertiseService("exampleMinimalService",
//                                                    &JointPositionController::serviceCallback,
//                                                    this);  
//     // add more services here, as needed
// }

// void JointPositionController::initializePublishers(){
//     // ROS_INFO("Initializing Publishers");
//     // nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
//     state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 1, true); 
// }


void JointPositionController::goalCallback(const sensor_msgs::JointState& msg) {

    // val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of JointPositionController can access it
    ROS_INFO("updating goal state");
    curr_goal_state_ = msg;
    curr_q_goal_ = Eigen::VectorXd::Map(&curr_goal_state_.position[0], 7);
}


void JointPositionController::setJointPositionGoal(std::array<double, 7>& q_goal){
    curr_q_goal_ = Vector7d(q_goal.data());
    //TODO: Must update curr_goal_state here too!!!!!!!!
}

franka::JointPositions JointPositionController::motion_generator_callback(const franka::RobotState& robot_state,
                                                                          franka::Duration period) {
    // int dt = period.toSec();
    double dt = 0.001;
    time_ += period.toSec();
    std::array<bool, 7> joint_motion_finished{};
    bool motion_finished;

    curr_robot_state_ = robot_state;

    // std::cout << "curr joint state object" << std::endl;
    // for(size_t i = 0; i < 7; ++i){
    //     std::cout << robot_state.q[i] << std::endl;
    // }


    curr_q_ = Vector7d(robot_state.q_d.data());
    
    delta_q_ = (curr_q_goal_ - curr_q_) / dt;
    // std::cout << dt << std::endl;
    // std::cout << "Current goal \n" << curr_q_goal_ << std::endl;
    // std::cout << "Measured q \n" << Vector7d(robot_state.q.data()) << std::endl;
    // std::cout << "Desired q \n" << Vector7d(robot_state.q_d.data()) << std::endl;

    // std::cout << "Unnormalized delta_q \n" << delta_q_ << std::endl;

    double max_delta_q = delta_q_.lpNorm<Eigen::Infinity>();
    // std::cout << "max_delta_q " <<  max_delta_q << "\t" << dq_max_ << std::endl;

    //Normalize delta_q to respect limits
    if(max_delta_q > dq_max_){
        for(size_t i = 0; i < 7; i++){
            delta_q_[i] = (delta_q_[i] / max_delta_q) * dq_max_;
        }
    }

    // std::cout << "Normalized delta_q \n" << delta_q_ << std::endl;


    // Check if motion is finished
    for (size_t i = 0; i < 7; i++) {
        if (std::abs(delta_q_[i]) <= kDeltaQMotionFinished) {
            // delta_q_[i] = 0.0;
            joint_motion_finished[i] = true;
        }
    }

    // calculate desired joint position 
    Vector7d q_desired = curr_q_ + (delta_q_ * dt);

    // std::cout << "q_curr" << curr_q_ << std::endl;
    // std::cout << "q_des" << q_desired << std::endl;

    std::array<double, 7> joint_positions;
    Eigen::VectorXd::Map(&joint_positions[0], 7) = q_desired;
    
    // std::cout << "Desired output \n" << q_desired << std::endl;

    franka::JointPositions output(joint_positions);
    
    if(!ros::ok()){
        ROS_INFO("Ros shutdown");
        motion_finished = true;
    }
    else{
        motion_finished = std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                                      [](bool x) { return x; });
    }    
    
    output.motion_finished = motion_finished;

    ros::spinOnce();

    return output;

  }

bool JointPositionController::read_state_callback(const franka::RobotState& robot_state){

    franka::Duration period(0);
    motion_generator_callback(robot_state, period);
    ros::spinOnce();
    return ros::ok();

}





// void JointPositionController::initiate_robot_control_loop(){
//     robot_.control(motion_generator_callback);
// }


void JointPositionController::setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}




// //member function implementation for a service callback function
// bool JointPositionController::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
//     ROS_INFO("service callback activated");
//     response.resp = true; // boring, but valid response info
//     return true;
// }




