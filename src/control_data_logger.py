#! /usr/bin/env python
import os
import numpy as np
import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import matplotlib.pyplot as plt
# import torch
# torch.multiprocessing.set_start_method('spawn',force=True)

import yaml

# from stochastic_control.mpc_tools.rollout.arm_reacher import ArmReacher
from stochastic_control.mpc_tools.rollout.arm_reacher_nn_collision import ArmReacherCollisionNN
np.set_printoptions(precision=2)
# x_des_list = [np.array([0.1643, -0.4172,  0.7743]),
#               np.array([-0.5441, -0.3595,  0.4602]), np.array([-1.0391, -0.5228,  0.1615]),
#               np.array([0.1643, -0.4172,  0.7743]),
#               np.array([0.9243, -0.7062,  0.1615])]


franka_bl_state = np.array([-0.45, 0.68, 0.0, -1.4, 0.0, 2.4,0.0,
                            0.0,0.0,0.0,0.0,0.0,0.0,0.0])
franka_br_state = np.array([0.45, 0.68, 0.0, -1.4, 0.0, 2.4,0.0,
                            0.0,0.0,0.0,0.0,0.0,0.0,0.0])
x_des_list = [franka_bl_state, franka_br_state]#, drop_state, home_state]




class DataLogger(object):
    def __init__(self, joint_states_topic, joint_command_topic, goal_topic, plot_freq, joint_names=[]):
        #user_joint_command_topic
        self.joint_states_topic = joint_states_topic
        self.joint_command_topic = joint_command_topic
        self.goal_topic = goal_topic
        self.plot_freq = plot_freq
        self.joint_names = joint_names

        self.fig, self.ax = plt.subplots(3,1)

        #Initialize ROS
        self.state_sub = rospy.Subscriber(self.joint_states_topic, JointState, self.state_callback)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback)
        self.rate = rospy.Rate(self.plot_freq)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #Variables for book-keeping
        self.curr_joint_state = None
        self.curr_ee_goal = None
        self.robot_q_list_r = []
        self.robot_qd_list_r = []

        self.ee_x = []; self.ee_y = []; self.ee_z = [];
        self.ee_quat_x = []; self.ee_quat_y = []; self.ee_quat_z = [];
        self.ee_quat_w = []

        self.tsteps = []
        self.curr_tstep = 0.0
        self.start_t = 0.0
        # self.robot_qd_des_list_r = []
        # self.robot_q_list_f = []
        # self.robot_qd_list_f = []
        # self.robot_qd_des_list_f = []
        # self.command_q_list = []
        # self.command_qdd_list = []
        # self.ee_dist_err_list = []
        # self.ee_rot_err_list = []

        self.ax[0].set_title('Robot Joint Positions')
        self.ax[1].set_title('Robot Joint Velocities')
        self.ax[2].set_title('End Effector Position')

    # def user_command_callback(self, msg):
    #     if msg.data == "stop":
    #         self.stop_controller = True
    #         rospy.loginfo('Stop command received')

    def state_callback(self, msg):
        self.curr_joint_state = msg

    def goal_callback(self, msg):
        self.curr_ee_goal = msg
 
    
    def loop(self):
        
        while not rospy.is_shutdown():
            if self.curr_joint_state is not None: # and self.curr_ee_goal is not None:
                    # self.log_data(mpc_qdd_des, [0., 0., 0.])
                self.robot_q_list_r.append(self.curr_joint_state.position)
                self.robot_qd_list_r.append(self.curr_joint_state.velocity)
                self.tsteps.append(self.curr_tstep)
                try:
                    ee_transform = self.tfBuffer.lookup_transform('panda_link0', 'ee_link', rospy.Time(0))
                    self.ee_x.append(ee_transform.transform.translation.x)
                    self.ee_y.append(ee_transform.transform.translation.y)
                    self.ee_z.append(ee_transform.transform.translation.z)
                    self.ee_quat_x.append(ee_transform.transform.rotation.x)
                    self.ee_quat_y.append(ee_transform.transform.rotation.y)
                    self.ee_quat_z.append(ee_transform.transform.rotation.z)
                    self.ee_quat_w.append(ee_transform.transform.rotation.w)
                except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('exception')
                    continue

            # self.robot_qd_des_list_r.append(self.curr_state_raw.effort)

                if self.curr_tstep == 0:
                    self.start_t = rospy.get_time()
                
                self.curr_tstep = rospy.get_time() - self.start_t


            else:
                rospy.loginfo('[DataLogger]: Waiting for state')
            
            # if self.curr_ee_goal is not None:
            #     pass

            # else:
            #     rospy.loginfo('[DataLogger]: Waiting for goal')

            self.rate.sleep()

        self.dump_data()
        self.plot_data()
    
    # def log_data(self,):
        
        # self.robot_q_list_f.append(self.curr_state_filtered.position)
        # self.robot_qd_list_f.append(self.curr_state_filtered.velocity)
        # self.robot_qd_des_list_f.append(self.curr_state_filtered.effort)
        
        # self.command_q_list.append(self.curr_mpc_command.position)
        # self.command_qdd_list.append(qdd_des)

        # self.ee_dist_err_list.append(ee_error[0])
        # self.ee_rot_err_list.append(ee_error[1])

    def dump_data(self):
        # print('Closing DataLogger')
        np.savez('/home/mohak/catkin_ws/src/franka_motion_control/data/mpc_data.npz', 'wb',
                q_robot_r= self.robot_q_list_r,
                qd_robot_r=self.robot_qd_list_r)
            # np.savez('/home/mohak/catkin_ws/src/franka_motion_control/data/mpc_data.npz', 'wb', 
            #         q_robot_r = self.robot_q_list_r, 
            #         qd_robot_r = self.robot_qd_list_r,
            #         qd_des_robot_r = self.robot_qd_des_list_r,
            #         q_robot_f = self.robot_q_list_f, 
            #         qd_robot_f = self.robot_qd_list_f,
            #         qd_des_robot_f = self.robot_qd_des_list_f,
            #         q_cmd = self.command_q_list,
            #         qdd_cmd = self.command_qdd_list,
            #         ee_dist_err = self.ee_dist_err_list,
            #         ee_rot_err = self.ee_rot_err_list)
        print('Logs dumped')
    
    def plot_data(self):
        self.ax[0].plot(self.tsteps, self.robot_q_list_r)
        self.ax[1].plot(self.tsteps, self.robot_qd_list_r)
        self.ax[2].plot(self.ee_x, 'r')
        self.ax[2].plot(self.ee_y, 'g')
        self.ax[2].plot(self.ee_z, 'b')
        plt.show()

    def joint_state_to_dict(self, msg):
        return {'position': np.array(msg.position), 
                'velocity': np.array(msg.velocity)}
    
    def dict_to_joint_state(self, dict):
        msg = JointState()
        msg.position = dict['position']
        msg.velocity = dict['velocity']
        return msg
    
    def pose_stamped_to_np(self, msg):
        pos = np.array([msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])
        quat = np.array([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])
        return pos, quat

if __name__ == '__main__':
    rospy.init_node("data_logger", anonymous=True)

    joint_states_topic = rospy.get_param('~joint_states_topic')
    joint_command_topic = rospy.get_param('~joint_command_topic')
    ee_goal_topic = rospy.get_param('~ee_goal_topic')
    joint_names = rospy.get_param('~joint_names')
    plot_freq = rospy.get_param('~plot_freq')

    data_logger = DataLogger(joint_states_topic,
                             joint_command_topic,
                             ee_goal_topic,
                             plot_freq,
                             joint_names)

    rospy.loginfo('[DataLogger]: Initialized')
    data_logger.loop()