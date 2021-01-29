#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import JointState


q_home = np.array([0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4]) 


def goal_pub():
    pub = rospy.Publisher('joint_pos_goal', JointState, queue_size=10)
    rospy.init_node('joint_pos_goal_publisher', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    JointState goal_state

    
    while not rospy.is_shutdown():
        curr_goal_q = q_home
        goal_state.position = curr_goal_q
        # rospy.loginfo(hello_str)
        pub.publish(goal_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        goal_pub()
    except rospy.ROSInterruptException:
        pass