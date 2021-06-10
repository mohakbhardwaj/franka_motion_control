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



class MoveitReacher(object):
    def __init__(self):
        self.robot_state_topic = "joint_states"
        self.ee_goal_topic = "ee_goal"
        self.log_data = True
        
        self.goal_ee_pos_list= [[ 3.8689e-01,  4.4958e-01,  5.9739e-01],
        [ 4.7659e-01,  4.9783e-01,  2.8616e-01],
        [ 5.7576e-01, -4.7868e-01,  5.7678e-01],
        [ 5.4563e-01, -4.2102e-01,  2.8616e-01],
        [ 4.7659e-01,  4.9783e-01,  2.8616e-01],
        [ 3.0835e-01,  5.2710e-08,  4.9148e-01]]
        
        self.goal_ee_quat_list = [[ 0.4595,  0.1146, -0.2016, -0.8573],
        [ 0.1133, -0.0171,  0.9911,  0.0680],
        [ 0.3932,  0.3132,  0.7399,  0.4470],
        [ 0.1254,  0.6598,  0.7397,  0.0417],
        [ 0.1133, -0.0171,  0.9911,  0.0680],
        [ 0.0000,  0.7055,  0.7087,  0.0000]]

        self.curr_goal_idx = 0
        self.num_goals = len(self.goal_ee_pos_list)
    
        #Setup Moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        # self.move_group_arm.set_planner_id("RRTstarkConfigDefault")
        self.move_group_arm.set_max_acceleration_scaling_factor(0.5)
        self.move_group_arm.set_max_velocity_scaling_factor(0.5)

        rospy.sleep(2)
        # self.move_group_hand = moveit_commander.MoveGroupCommander("hand")
        self.table_pose = PoseStamped()
        self.table_pose.header.frame_id = self.robot.get_planning_frame()
        self.table_pose.pose.position.x = 0.
        self.table_pose.pose.position.y = 0.
        self.table_pose.pose.position.z = 0.03
        self.scene.add_box("table", self.table_pose, (2.0, 2.0, 0.2 ))        
        
        # self.scene.add_box('table', 2.0, 2.0, 0.2, 0.0, 0.0, 0.03)

        # pose: [0.0, 0.0, 0.03, 0, 0, 0, 1.0] # x, y, z, qx, qy, qz, qw
        self.move_group_arm.set_planning_time(10.0)
        print(self.move_group_arm.get_planning_time())
        #Setup ROS
        self.curr_robot_state = self.robot.get_current_state()
        self.curr_robot_state_ros = PoseStamped()
        self.curr_ee_goal_pos = self.goal_ee_pos_list[0]
        self.curr_ee_goal_quat = self.goal_ee_quat_list[0]
        self.curr_ee_goal = PoseStamped()
        self.curr_ee_goal.pose.position.x = self.curr_ee_goal_pos[0]
        self.curr_ee_goal.pose.position.y = self.curr_ee_goal_pos[1]
        self.curr_ee_goal.pose.position.z = self.curr_ee_goal_pos[2]
        self.curr_ee_goal.pose.orientation.w = self.curr_ee_goal_quat[0]
        self.curr_ee_goal.pose.orientation.x = self.curr_ee_goal_quat[1]
        self.curr_ee_goal.pose.orientation.y = self.curr_ee_goal_quat[2]
        self.curr_ee_goal.pose.orientation.z = self.curr_ee_goal_quat[3]


        self.rate = rospy.Rate(100)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.state_sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)


        self.executing = False
        rospy.loginfo("Moveit Reacher initialized")

        self.tstep = 0
        self.start_t = 0
        self.last_update_tstep = 0
        self.plan_time = 0
        if self.log_data:
            self.log_file = '/home/mohak/catkin_ws/src/franka_motion_control/data/moveit_data_{}.npz'.format(datetime.now())
            self.tsteps = []
            self.robot_q_list = []
            self.robot_qd_list = []
            self.ee_goal_pos_list = []
            self.ee_goal_quat_list = []
            self.plan_times = []
            self.execute_times = []


    def state_callback(self, msg):
        self.curr_robot_state = msg
        if self.tstep == 0:
            self.start_t = rospy.get_time()
        self.tstep = rospy.get_time() - self.start_t
        self.robot_q_list.append(self.curr_robot_state.position)
        self.robot_qd_list.append(self.curr_robot_state.velocity)
        self.ee_goal_pos_list.append(self.curr_ee_goal_pos)
        self.ee_goal_quat_list.append(self.curr_ee_goal_quat)
        self.tsteps.append(self.tstep)

    def control_loop(self):
        while True:

            self.move_group_arm.set_start_state_to_current_state()
            self.move_group_arm.set_pose_target(self.curr_ee_goal.pose)
            
            time_before = rospy.get_time()
            plan = self.move_group_arm.plan()
            print(plan)
            self.plan_times.append(rospy.get_time() - time_before)
            
            time_before = rospy.get_time()
            self.move_group_arm.execute(plan, wait=True)
            self.execute_times.append(rospy.get_time() - time_before)
            
            self.move_group_arm.stop()
            self.move_group_arm.clear_pose_targets()

            self.update_goal_from_sequence()
            
                                                            
            # if self.log_data:
            #     self.append_data()

            # self.rate.sleep() 
    
    def update_goal_from_sequence(self):
        self.curr_goal_idx = (self.curr_goal_idx + 1) % (self.num_goals)
        self.curr_ee_goal_pos = self.goal_ee_pos_list[self.curr_goal_idx]
        self.curr_ee_goal_quat = self.goal_ee_quat_list[self.curr_goal_idx]
        self.curr_ee_goal.pose.position.x = self.curr_ee_goal_pos[0]
        self.curr_ee_goal.pose.position.y = self.curr_ee_goal_pos[1]
        self.curr_ee_goal.pose.position.z = self.curr_ee_goal_pos[2]
        self.curr_ee_goal.pose.orientation.w = self.curr_ee_goal_quat[0]
        self.curr_ee_goal.pose.orientation.x = self.curr_ee_goal_quat[1]
        self.curr_ee_goal.pose.orientation.y = self.curr_ee_goal_quat[2]
        self.curr_ee_goal.pose.orientation.z = self.curr_ee_goal_quat[3]


    def append_data(self):
        self.robot_q_list.append(self.curr_state.position)
        self.robot_qd_list.append(self.curr_state.velocity)
        self.ee_goal_pos_list.append(self.curr_ee_goal_pos)
        self.ee_goal_quat_list.append(self.curr_ee_goal_quat)
        self.tsteps.append(self.tstep)


    def close(self):
        print('Closing')
        self.move_group_arm.stop()

        if self.log_data:
            print('Logging data')
            try:
                np.savez(self.log_file, 'wb', 
                        q_robot = self.robot_q_list, 
                        qd_robot = self.robot_qd_list,
                        ee_goal_pos = self.ee_goal_pos_list,
                        ee_goal_quat = self.ee_goal_quat_list,
                        plan_times = self.plan_times,
                        execute_times = self.execute_times,
                        tsteps = self.tsteps)
            except Exception:
                print('Unable to save logs')

            print('Logs dumped')
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_reacher", anonymous=True, disable_signals=True)
    print('Initializing reacher')
    reacher = MoveitReacher()
    rospy.loginfo("WARNING: This example will move the robot! \n"
                 "Please make sure to have the user stop button at hand! \n"
                 "Press Enter to continue... \n")
    raw_input('')
    try:
        reacher.control_loop()
    except KeyboardInterrupt:
        reacher.close()
