// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_pos_goal_publisher");
  ros::NodeHandle node_handle("~");

  std::vector<std::string> joint_names;
  node_handle.getParam("joint_names", joint_names);

  std::array<double, 7> q_home = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}; 
  std::array<std::array<double, 7>, 2> goal_configs = {{{-M_PI_4, -M_PI_4, 0, - M_PI, 0, M_PI_2, M_PI_4}, 
                                                        {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}};


  double publish_rate;
  node_handle.getParam("publish_rate", publish_rate);

  ROS_INFO("%d", publish_rate);


  ros::Rate rate(publish_rate);

  sensor_msgs::JointState q_goal_msg;
  q_goal_msg.effort.resize(joint_names.size());
  q_goal_msg.name.resize(joint_names.size());
  q_goal_msg.position.resize(joint_names.size());
  q_goal_msg.velocity.resize(joint_names.size());

  for (size_t i = 0; i < joint_names.size(); i++) {
    q_goal_msg.name[i] = joint_names[i];
  }

  ros::Publisher publisher = node_handle.advertise<sensor_msgs::JointState>("joint_pos_goal", 1);

  while (ros::ok()){
    q_goal_msg.header.stamp = ros::Time::now();
    for (size_t i = 0; i < joint_names.size(); i++) {
        q_goal_msg.position[i] = q_home[i];
        q_goal_msg.velocity[i] = 0.0;
        q_goal_msg.effort[i] = 0.0;
    }
    publisher.publish(q_goal_msg);
    ros::spinOnce();
    rate.sleep();
  }    


  return 0;
}
