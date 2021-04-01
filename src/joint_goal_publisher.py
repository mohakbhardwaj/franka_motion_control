#!/usr/bin/env python
from copy import deepcopy
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

M_PI = np.pi
M_PI_2 = np.pi / 2.0
M_PI_4 = np.pi / 4.0
M_PI_8 = np.pi/8.0
colors = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#e6ab02', '#a6761d']

joint_to_increment = 2
delta_increment = 0.01

q_list = []
qd_list = []
tstep_list = []
tstep = 0
start_t = 0
start_q = None
goal_q = None
goal_qd = np.zeros(7)

def state_sub(msg):
    global tstep, start_t, start_q, goal_q
    q_list.append(msg.position)
    qd_list.append(msg.velocity)
    if tstep == 0:
        start_q = deepcopy(np.array(msg.position))
        start_t = rospy.get_time()
        goal_q = start_q
        print(goal_q)
        goal_q[joint_to_increment] += delta_increment

    tstep = rospy.get_time() - start_t
    tstep_list.append(tstep)


def goal_pub():
    global start_q, goal_q, goal_qd, joint_to_increment, delta_increment
    pub = rospy.Publisher('franka_motion_control/joint_command', JointState, queue_size=1)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if start_q is not None:
            goal_state = JointState()

            goal_state.position = goal_q
            goal_state.velocity = goal_qd
            print("Goal pos: {}, Goal vel: {}".format(goal_q, goal_qd))
            pub.publish(goal_state)
        else:
            print('Waiting for state....')
        
        rate.sleep()
    plot()

def plot():
    global goal_q, goal_qd, q_list, qd_list, joint_to_increment, tstep_list
    fig, ax = plt.subplots(2,1)
    num_pts = len(tstep_list)
    q_list = np.array(q_list)
    qd_list = np.array(qd_list)
    if num_pts > 1:
        # for i in range(7):
        i = joint_to_increment
        ax[0].plot(tstep_list, [goal_q[i]]*num_pts, linestyle='dashed', color=colors[i], label='joint_{}_des'.format(i))
        ax[0].plot(tstep_list, q_list[:,i], color=colors[i])
        ax[1].plot(tstep_list, [goal_qd[i]]*num_pts, linestyle='dashed', color=colors[i])
        ax[1].plot(tstep_list, qd_list[:,i], color=colors[i])
        ax[0].set_title('Joint Position')
        ax[1].set_title('Joint Velocity')
        ax[0].legend()
        plt.show()



if __name__ == '__main__':
    rospy.init_node('joint_pos_goal_publisher', anonymous=True)
    state_sub = rospy.Subscriber('franka_motion_control/joint_states', JointState, state_sub)
    goal_pub()