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

class MoveitReacher(object):
    def __init__(self, robot_state_topic, ee_goal_topic):
        self.robot_state_topic = robot_state_topic
        self.ee_goal_topic = ee_goal_topic
        #Setup Moveit
        self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        # self.move_group_hand = moveit_commander.MoveGroupCommander("hand")

        
        
        #Setup ROS
        self.curr_joint_state = None
        self.curr_ee_goal = None
        self.rate = rospy.Rate(100)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.state_sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)
        self.goal_sub = rospy.Subscriber(self.ee_goal_topic, PoseStamped, self.goal_callback)


        self.executing = False
        rospy.loginfo("Moveit Reacher initialized")
    


    def state_callback(self, msg):
        self.curr_joint_state = msg



    def goal_callback(self, msg):
        if not self.executing:
            self.curr_ee_goal = msg
            self.move_group_arm.set_pose_target(self.curr_ee_goal.pose)
            self.move_group_arm.go(wait=True)
            self.executing = True
            self.move_group_arm.stop()
            self.move_group_arm.clear_pose_targets()

    def control_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep() 

if __name__ == '__main__':
    rospy.init_node("moveit_reacher", anonymous=True)
    robot_state_topic = rospy.get_param('~joint_states_topic', 'joint_states')
    ee_goal_topic = rospy.get_param('~ee_goal_topic', 'ee_goal')

    reacher = MoveitReacher(robot_state_topic, ee_goal_topic)
    reacher.control_loop()
