#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import JointState

M_PI = np.pi
M_PI_2 = np.pi / 2.0
M_PI_4 = np.pi / 4.0


q_home = np.array([0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4]) 

q_goals = np.array([[0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4],
                    [M_PI_4, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4]])

num_goals = q_goals.shape[0]
curr_goal_idx = 0
curr_goal_q = q_goals[curr_goal_idx]


def state_callback(data):
    global curr_goal_idx, curr_goal_q, num_goals
    # print('curr robot joint pos')
    curr_robot_pos = data.position
    # print(np.all(np.abs(curr_robot_pos - curr_goal_q) < 1e-6), curr_goal_idx)
    if np.all(np.abs(curr_robot_pos - curr_goal_q) < 1e-6):
        curr_goal_idx = ((curr_goal_idx+1) % num_goals)
        curr_goal_q = q_goals[curr_goal_idx] 

def goal_pub():
    goal_state = JointState()
    while not rospy.is_shutdown():
        goal_state.position = curr_goal_q
        print("Curr goal: ", curr_goal_q)
        pub.publish(goal_state)
        rate.sleep()

if __name__ == '__main__':
    pub = rospy.Publisher('joint_pos_controller/joint_pos_goal', JointState, queue_size=1)
    sub = rospy.Subscriber("joint_pos_controller/joint_states", JointState, state_callback)
    rospy.init_node('joint_pos_goal_publisher', anonymous=True)
    # rate = rospy.Rate(100)
