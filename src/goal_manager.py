#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import std_msgs.msg
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

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

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("goal_marker")
        
        # create an interactive marker for our server
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "panda_link0"
        self.int_marker.name = "goal_marker"
        self.int_marker.description = "End-effector Goal"
        self.int_marker.scale = 0.1
        self.int_marker.pose.position = self.curr_goal.pose.position


        # create a grey box marker
        self.box_marker = Marker()
        self.box_marker.type = Marker.SPHERE
        self.box_marker.scale.x = 0.1
        self.box_marker.scale.y = 0.1
        self.box_marker.scale.z = 0.1
        self.box_marker.color.r = 0.0
        self.box_marker.color.g = 0.5
        self.box_marker.color.b = 0.5
        self.box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        self.box_control = InteractiveMarkerControl()
        self.box_control.always_visible = True
        self.box_control.markers.append(self.box_marker)

        # add the control to the interactive marker
        self.int_marker.controls.append(self.box_control)

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        self.move_x_control = InteractiveMarkerControl()
        self.move_x_control.name = "move_x"
        self.move_x_control.orientation.w = 1
        self.move_x_control.orientation.x = 1
        self.move_x_control.orientation.y = 0
        self.move_x_control.orientation.z = 0   
        self.move_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        self.int_marker.controls.append(self.move_x_control)

        self.move_y_control = InteractiveMarkerControl()
        self.move_y_control.name = "move_y"
        self.move_y_control.orientation.w = 1
        self.move_y_control.orientation.x = 0
        self.move_y_control.orientation.y = 0
        self.move_y_control.orientation.z = 1                
        self.move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.int_marker.controls.append(self.move_y_control)

        self.move_z_control = InteractiveMarkerControl()
        self.move_z_control.name = "move_z"
        self.move_z_control.orientation.w = 1
        self.move_z_control.orientation.x = 0
        self.move_z_control.orientation.y = 1
        self.move_z_control.orientation.z = 0
        self.move_z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.int_marker.controls.append(self.move_z_control)




        # add the interactive marker to our collection &
        # tell the server to call marker_callback() when feedback arrives for it
        self.server.insert(self.int_marker, self.marker_callback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()


    def marker_callback(self, msg):
        # p = msg.pose.position
        # q = msg.pose.orientation
        # print(msg.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))
        # print(msg.marker_name + "orientation is " + str(q.x) + ", " + str(q.y) + ", " + str(q.z) + ", " + str(q.w))
        self.curr_goal.header = msg.header
        self.curr_goal.pose = msg.pose



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