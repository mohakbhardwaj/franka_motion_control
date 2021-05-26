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
    def __init__(self):
        self.robot_state_topic = "joint_states"
        self.ee_goal_topic = "ee_goal"
        #Setup Moveit
        self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        # self.move_group_hand = moveit_commander.MoveGroupCommander("hand")
        self.log_data = False
        
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
        
        #Setup ROS
        self.curr_state = None
        self.curr_ee_goal_pos = self.goal_ee_pos_list[0]
        self.curr_ee_goal_quat = self.goal_ee_quat_list[0]
        self.curr_ee_goal = PoseStamped()
        self.curr_ee_goal.pose.position.x = self.curr_ee_goal_pos[0]
        self.curr_ee_goal.pose.position.y = self.curr_ee_goal_pos[1]
        self.curr_ee_goal.pose.position.z = self.curr_ee_goal_pos[2]
        self.curr_ee_goal.pose.position.w = self.curr_ee_goal_pos[0]
        self.curr_ee_goal.pose.position.x = self.curr_ee_goal_pos[1]
        self.curr_ee_goal.pose.position.y = self.curr_ee_goal_pos[2]
        self.curr_ee_goal.pose.position.z = self.curr_ee_goal_pos[3]


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
        if self.log_data:
            self.log_file = '/home/mohak/catkin_ws/src/franka_motion_control/moveit_data_{}.npz'.format(datetime.now())
            self.tsteps = []
            self.robot_q_list = []
            self.robot_qd_list = []
            self.ee_goal_pos_list = []
            self.ee_goal_quat_list = []
    
    def state_callback(self, msg):
        self.curr_joint_state = msg

    # def goal_callback(self, msg):
    #     self.curr_ee_goal = msg
    #     self.curr_ee_goal_pos, self.curr_ee_goal_quat = self.pose_stamped_to_np(msg)


    def control_loop(self):
        while not rospy.is_shutdown():
            self.move_group_arm.set_pose_target(self.curr_ee_goal.pose)
            self.move_group_arm.go(wait=True)
            self.executing = True

            if self.tstep == 0:
                self.start_t = rospy.get_time()
            self.tstep = rospy.get_time() - self.start_t
                                                                
            if self.log_data:
                self.append_data()

            if self.tstep - self.last_update_tstep >= 12.0:
                self.move_group_arm.stop()
                self.move_group_arm.clear_pose_targets()
                self.update_goal_from_sequence()


            self.rate.sleep() 
    

    def append_data(self):
        self.robot_q_list.append(self.curr_state.position)
        self.robot_qd_list.append(self.curr_state.velocity)
        self.ee_goal_pos_list.append(self.curr_ee_goal_pos)
        self.ee_goal_quat_list.append(self.curr_ee_goal_quat)
        self.tsteps.append(self.tstep)

    def close(self):
        print('Closing')
        if self.log_data:
            print('Logging data')
            try:
                np.savez(self.log_file, 'wb', 
                        q_robot = self.robot_q_list, 
                        qd_robot = self.robot_qd_list,
                        qd_des_robot = self.robot_qd_des_list,
                        q_cmd = self.command_q_list,
                        qd_cmd = self.command_qd_list,
                        qdd_cmd = self.command_qdd_list,
                        ee_goal_pos = self.ee_goal_pos_list,
                        ee_goal_quat = self.ee_goal_quat_list,
                        tsteps = self.tsteps)
            except e:
                print('Unable to save logs')

            print('Logs dumped')
if __name__ == '__main__':
    rospy.init_node("moveit_reacher", anonymous=True, disable_signals=True)
    reacher = MoveitReacher()
    reacher.control_loop()
