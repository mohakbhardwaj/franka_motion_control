#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

M_PI = np.pi
M_PI_2 = np.pi / 2.0
M_PI_4 = np.pi / 4.0
M_PI_8 = np.pi/8.0

joint_to_tune = 4
increment = 0.1
# q_goal = np.array([0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4,\
#                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#Home
# q_goal = np.array([0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4,\
#                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# q_goal[joint_to_tune] += increment

#Don't send here
# q_goal = np.array([-0.45, 0.68, 0.0, -1.4, 0.0, 2.4,0.0,
#                             0.0,0.0,0.0,0.0,0.0,0.0,0.0])

#Send here
q_goal = np.array([-0.45, 0.3, 0.0, -1.4, 0.0, 2.4,0.0,
                   0.0,0.0,0.0,0.0,0.0,0.0,0.0])

q_list = []
qd_list = []

colors = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#e6ab02', '#a6761d']

def state_sub(msg):
    q_list.append(msg.position)
    qd_list.append(msg.velocity)

def goal_pub():
    goal_state = JointState()
    pub = rospy.Publisher('joint_pos_controller/joint_command', JointState, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        goal_state.position = q_goal[0:7]
        goal_state.velocity = q_goal[7:14]
        print("Curr goal: ", q_goal)
        pub.publish(goal_state)
        rate.sleep()
    plot()

def plot():
    global q_list, qd_list, joint_to_tune
    fig, ax = plt.subplots(2,1)
    num_pts = len(q_list)
    q_list = np.array(q_list)
    qd_list = np.array(qd_list)
    if num_pts > 1:
        # for i in range(7):
        i = joint_to_tune
        ax[0].plot(range(num_pts), [q_goal[i]]*num_pts, linestyle='dashed', color=colors[i], label='joint_{}_des'.format(i))
        ax[0].plot(range(num_pts), q_list[:,i], color=colors[i])
        ax[1].plot(range(num_pts), [q_goal[i+7]]*num_pts, linestyle='dashed', color=colors[i])
        ax[1].plot(range(num_pts), qd_list[:,i], color=colors[i])
        ax[0].set_title('Joint Position')
        ax[1].set_title('Joint Velocity')
        ax[0].legend()
        plt.show()



if __name__ == '__main__':
    rospy.init_node('joint_pos_goal_publisher', anonymous=True)
    state_sub = rospy.Subscriber('joint_pos_controller/joint_states', JointState, state_sub)
    goal_pub()