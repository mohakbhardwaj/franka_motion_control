#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
import random
import numpy as np
random.seed(0)
np.random.seed(0)
from datetime import datetime
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal
import geometry_msgs.msg
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
    GripperCommandGoal,
)



class MoveitGripper(object):
    def __init__(self):
        self.robot_state_topic = "joint_states"
            
        #Setup Moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group_hand = moveit_commander.MoveGroupCommander("hand")

        self.gripper_action_client = actionlib.SimpleActionClient(
            '/franka_gripper/gripper_action',
            GripperCommandAction,
        )        
        self.gripper_grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', 
            GraspAction,
        )
        self.gripper_grasp_client.wait_for_server()
        self.gripper_state = 'open'



        self.state_sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)


        self.executing = False
        rospy.loginfo("Moveit Reacher initialized")

        self.curr_robot_state = None
        self.tstep = 0
        self.start_t = 0
        self.last_update_tstep = 0
        self.plan_time = 0
        # if self.log_data:
        #     self.log_file = '/home/mohak/catkin_ws/src/franka_motion_control/data/moveit_data_{}.npz'.format(datetime.now())
        #     self.tsteps = []
        #     self.robot_q_list = []
        #     self.robot_qd_list = []
        #     self.ee_goal_pos_list = []
        #     self.ee_goal_quat_list = []
        #     self.plan_times = []
        #     self.execute_times = []


    def state_callback(self, msg):
        self.curr_robot_state = msg
        if self.tstep == 0:
            self.start_t = rospy.get_time()
        # self.tstep = rospy.get_time() - self.start_t
        # self.robot_q_list.append(self.curr_robot_state.position)
        # self.robot_qd_list.append(self.curr_robot_state.velocity)
        # self.ee_goal_pos_list.append(self.curr_ee_goal_pos)
        # self.ee_goal_quat_list.append(self.curr_ee_goal_quat)
        # self.tsteps.append(self.tstep)

    def open(self):
        goal = GripperCommandGoal()
        goal.command.position = 0.01
        goal.command.max_effort = 0.1 #self.cfg['manipulation']['gripper_max_effort']          
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result(rospy.Duration.from_sec(3.0))        # if self.gripper_state == 'close':
        #     self.gripper_command(0.02)
        #     self.gripper_state = 'open'

    def close(self):
        # if self.gripper_state == 'open':
        #     self.gripper_command(0.00001)
        #     self.gripper_state = 'close'        
        goal = GraspGoal()
        goal.width = 0.0001
        goal.speed = 0.7
        goal.force = 30
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.05        
        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result(rospy.Duration.from_sec(3.0))
            
   

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_gripper", anonymous=True, disable_signals=True)
    print('Initializing Gripper')
    gripper = MoveitGripper()
    
    while True:
        try:
            raw_input('Press enter to open gripper')
            gripper.open()
            raw_input('Press enter to close gripper')
            gripper.close()
        except KeyboardInterrupt:
            exit()

