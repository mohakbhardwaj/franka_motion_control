#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import std_msgs.msg
from differentiable_robot_model.differentiable_robot_model import DifferentiableRobotModel
from differentiable_robot_model.coordinate_transform import matrix_to_quaternion, quaternion_to_matrix
import numpy as np
import torch

# goal_ee_pos_list = [[-0.3554, -0.7373,  0.2]] #0.1479
# goal_ee_quat_list = [[ 0.1571,  0.8050, -0.5714,  0.0267]]

franka_bl_state = np.array([-0.45, 0.68, 0.0, -1.4, 0.0, 2.4,0.0,
                            0.0,0.0,0.0,0.0,0.0,0.0,0.0])
franka_br_state = np.array([0.45, 0.68, 0.0, -1.4, 0.0, 2.4,0.0,
                            0.0,0.0,0.0,0.0,0.0,0.0,0.0])
q_des_list = [franka_bl_state, franka_br_state]#, drop_state, home_state]



class GoalManager(object):
    def __init__(self, sub_topic, pub_topic, urdf_path, publish_freq=10):
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.urdf_path = urdf_path
        self.publish_freq = publish_freq
    
        self.pub = rospy.Publisher(self.pub_topic, PoseStamped, queue_size=1, latch=False)
        self.sub = rospy.Subscriber(self.sub_topic, JointState, self.goal_callback)
        self.rate = rospy.Rate(self.publish_freq)
        self.tensor_args = {'device':"cpu", 'dtype':torch.float32}

        self.robot_model = DifferentiableRobotModel(self.urdf_path, None, 
                            tensor_args=self.tensor_args)
        
        self.goal_state = torch.as_tensor(q_des_list[0], **self.tensor_args).unsqueeze(0)
        self.n_dofs = 7
        
        # Should update cartesian pose?
        self.goal_ee_pos, self.goal_ee_rot = self.robot_model.compute_forward_kinematics(self.goal_state[:,0:self.n_dofs], self.goal_state[:,self.n_dofs:self.n_dofs*2], link_name='panda_link8')
        self.goal_ee_quat = matrix_to_quaternion(self.goal_ee_rot)

        #TBD: Change to None after state machine node has been written
        self.curr_goal = PoseStamped()
        self.curr_goal.header = std_msgs.msg.Header()
        self.curr_goal.header.stamp = rospy.Time.now()
        self.curr_goal.pose.position.x = self.goal_ee_pos[0][0]
        self.curr_goal.pose.position.y = self.goal_ee_pos[0][1]
        self.curr_goal.pose.position.z = self.goal_ee_pos[0][2]
        self.curr_goal.pose.orientation.x = self.goal_ee_quat[0][0]
        self.curr_goal.pose.orientation.y = self.goal_ee_quat[0][1]
        self.curr_goal.pose.orientation.z = self.goal_ee_quat[0][2]
        self.curr_goal.pose.orientation.w = self.goal_ee_quat[0][3]



        # self.curr_goal.pose.position.x = goal_ee_pos_list[0][0]
        # self.curr_goal.pose.position.y = goal_ee_pos_list[0][1]
        # self.curr_goal.pose.position.z = goal_ee_pos_list[0][2]
        # self.curr_goal.pose.orientation.x = goal_ee_quat_list[0][0]
        # self.curr_goal.pose.orientation.y = goal_ee_quat_list[0][1]
        # self.curr_goal.pose.orientation.z = goal_ee_quat_list[0][2]
        # self.curr_goal.pose.orientation.w = goal_ee_quat_list[0][3]


    def goal_callback(self, msg):
        # self.curr_goal = msg
        pass
    
    def publish_goal_loop(self):
        while not rospy.is_shutdown():
        
            if self.curr_goal is not None:
                # rospy.loginfo('[Goal Manager] Curr Goal:', self.curr_goal)
                self.pub.publish(self.curr_goal)                
                self.rate.sleep()
            else:
                # rospy.loginfo('Waiting for goal...')
                pass

if __name__ == '__main__':
    rospy.init_node("goal_manager", anonymous=True)

    # mpc_yml_file = join_path(mpc_configs_path(), robot_params['mpc_yml'])
    joint_states_topic = rospy.get_param('~joint_states_topic')
    ee_goal_topic = rospy.get_param('~ee_goal_topic')
    publish_freq = rospy.get_param('~goal_pub_freq')
    urdf_path = rospy.get_param('~urdf_path')
 
    goal_manager = GoalManager(joint_states_topic,
                               ee_goal_topic,
                               urdf_path,
                               publish_freq)
    
    rospy.loginfo('Initiating Goal Publisher')

    goal_manager.publish_goal_loop()