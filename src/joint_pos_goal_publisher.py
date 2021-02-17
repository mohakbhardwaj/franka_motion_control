#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

M_PI = np.pi
M_PI_2 = np.pi / 2.0
M_PI_4 = np.pi / 4.0


q_goal = np.array([0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4,\
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def goal_pub():
    goal_state = JointState()
    pub = rospy.Publisher('joint_pos_controller/joint_pos_goal', JointState, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        goal_state.position = q_goal[0:7]
        goal_state.velocity = q_goal[7:14]
        print("Curr goal: ", q_goal)
        pub.publish(goal_state)
        rate.sleep()
    plot()


    




if __name__ == '__main__':
    rospy.init_node('joint_pos_goal_publisher', anonymous=True)
    print("here")
    goal_pub()