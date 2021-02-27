#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
import std_msgs.msg
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from differentiable_robot_model.differentiable_robot_model import DifferentiableRobotModel
from differentiable_robot_model.coordinate_transform import matrix_to_quaternion, quaternion_to_matrix
from stochastic_control.mpc_tools.cost import PoseCost
import numpy as np
import torch
import tf2_ros

# goal_ee_pos_list = [[-0.3554, -0.7373,  0.2]] #0.1479
# goal_ee_quat_list = [[ 0.1571,  0.8050, -0.5714,  0.0267]]

# franka_bl_state = np.array([-0.45, 0.68, 0.0, -1.4, 0.0, 2.4,0.0,
#                             0.0,0.0,0.0,0.0,0.0,0.0,0.0])

# franka_bl_state = np.array([-0.45, 0.4, 0.0, -1.4, 0.0, 2.4,0.0,
#                             0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# franka_br_state = np.array([0.45, 0.4, 0.0, -1.4, 0.0, 2.4,0.0,
#                             0.0,0.0,0.0,0.0,0.0,0.0,0.0])

# franka_bl_state = np.array([-0.85, 0.35, 0.2, -1.8, 0.0, 2.4,0.0,
#                                0.0,0.0,0.0,0.0,0.0,0.0,0.0])

# franka_br_state = np.array([1., 0.35, -0.2, -1.8, 0.0, 2.4,0.0,
#                                0.0,0.0,0.0,0.0,0.0,0.0,0.0])

# franka_b_state = np.array([0.0, -0.35, 0.0, -2.2, 0.0, 2.4,0.0,
#                               0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# q_des_list = [franka_bl_state, franka_br_state, franka_b_state]#, drop_state, home_state]

# q_des_list = [np.array([0.0, -0.7853, 0.0, -2.356, 0.0, 3.14, -0.785, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])]
# q_des_list = [np.array([-0.207, -0.494, -0.398, -2.239, 1.289, 1.194, 0.462, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])]

home_config = np.array([0.0, -np.pi/4.0, 0.0, -3 * (np.pi/4.0), 0.0, np.pi/2.0, np.pi/4.0]) 

# home_config = np.array([-0.207, -0.494, -0.398, -2.239, 1.289, 1.194, 0.462]) #for rotation constraint


# joint_goal_1 = np.array([-0.653, 0.668, -0.366, -1.464, 0.080, 2.168, 0.780, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# joint_goal_2 = np.array([-0.653, -0.165, -0.302, -1.876, 0.080, 1.878, 0.016, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# joint_goal_3 = np.array([1.003, 0.494, -0.302, -1.101, 0.080, 3.100, 0.685, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# joint_goal_4 = np.array([1.003, -0.475, 0.589, -1.975, 0.239, 1.277, 0.685, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# joint_goal_5 = np.array([0.525, 0.630, 0.621, -1.612, -0.366, 2.437, 0.685, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
# # q_des_list = [joint_goal_1, joint_goal_2, joint_goal_3, joint_goal_4, home_config]

# rot_const_goal_quat = np.array([1.2207e-04, -7.0739e-01,  7.0683e-01,  1.2207e-04])


joint_goal_1 = np.array([0.2, 0.4,0.5,-1.9,2.3,0.9,-1.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
joint_goal_2 = np.array([1.0, 0.35, -0.2, -1.8, 0.0, 2.4,0.0,
                            0.0,0.0,0.0,0.0,0.0,0.0,0.0])

joint_goal_3 = np.array([-1.4, 0.4, 0.68, -1.2, -1.5, 2.7, -1.1, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])
joint_goal_4 = np.array([-1.5, 0.6, 0.9, -1.2, 0.8, 2.00, -1, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])

joint_goal_5 = np.array([-0.2, 0.8, 1.3, -1.0, -0.7, 2.7, 0.685, 0.0,0.0,0.0,0.0,0.0,0.0,0.0])

joint_goal_6 = np.array([-0.85, 0.35, 0.2, -1.8, 0.0, 2.4,0.0,
                        0.0,0.0,0.0,0.0,0.0,0.0,0.0])

joint_goal_7 = np.array([1., 0.55, -0.2, -1.8, 0.0, 2.4,0.0,
                        0.0,0.0,0.0,0.0,0.0,0.0,0.0])
home_state = np.array([0.00, -0.78, 0.00, -2.35, 0.00, 1.57, 0.78,
                            0.0,0.0,0.0,0.0,0.0,0.0,0.0])
q_des_list = [joint_goal_1, joint_goal_2, joint_goal_4,
                     joint_goal_6, joint_goal_7, home_state]


class GoalManager(object):
    def __init__(self, state_sub_topic, goal_pub_topic, ee_pose_pub_topic, urdf_path, 
                 home_config, goal_sub_topic=None, publish_freq=10, mode='ee_goal_seq',
                 goal_pos_thresh=0.0, goal_rot_thresh=0.0):
        self.state_sub_topic = state_sub_topic
        self.goal_pub_topic = goal_pub_topic
        self.ee_pose_pub_topic = ee_pose_pub_topic
        self.goal_sub_topic = goal_sub_topic
        self.urdf_path = urdf_path
        self.home_config = home_config
        self.publish_freq = publish_freq
        self.mode = mode
        self.goal_pos_thresh = goal_pos_thresh
        self.goal_rot_thresh = goal_rot_thresh
        self.n_dofs = 7

        if self.home_config.shape[0] < 2*self.n_dofs:
            self.home_config = np.concatenate((self.home_config, np.zeros(self.n_dofs)))

        self.tensor_args = {'device':"cpu", 'dtype':torch.float32}
        self.robot_model = DifferentiableRobotModel(self.urdf_path, None, 
                            tensor_args=self.tensor_args)
        if self.mode == 'ee_goal_seq' or self.mode == 'ee_pose_seq':
            self.num_goals = len(q_des_list)
            self.curr_goal_idx = 0
            self.goal_state = torch.as_tensor(q_des_list[self.curr_goal_idx], **self.tensor_args).unsqueeze(0)
        elif self.mode == 'interactive_marker':
            self.goal_state = torch.as_tensor(self.home_config, **self.tensor_args).unsqueeze(0)
        elif self.mode == 'perception_tracker':
            self.goal_state = torch.as_tensor(self.home_config, **self.tensor_args).unsqueeze(0)
        else:
            raise rospy.logerror('[GoalManager]: Invalid mode')

        #Setup ROS
        self.goal_pub = rospy.Publisher(self.goal_pub_topic, PoseStamped, queue_size=1, latch=False)
        self.ee_pub = rospy.Publisher(self.ee_pose_pub_topic, PoseStamped, queue_size=1, latch=False)
        self.mpc_command_sub = rospy.Subscriber('joint_pos_controller/joint_command', JointState, self.mpc_command_callback)
        self.state_sub = rospy.Subscriber(self.state_sub_topic, JointState, self.state_callback)
        if (self.goal_sub_topic is not None) and (self.mode == 'perception_tracker'):
            self.goal_transform_sub = rospy.Subscriber(self.goal_sub_topic, PoseStamped, self.goal_transform_callback)
        self.rate = rospy.Rate(self.publish_freq)

        self.curr_goal_ros = PoseStamped()
        self.curr_goal_ros.header = std_msgs.msg.Header()
        self.curr_goal_ros.header.stamp = rospy.Time.now()

        if self.goal_state is not None:
            self.goal_ee_pos, self.goal_ee_rot = self.robot_model.compute_forward_kinematics(self.goal_state[:,0:self.n_dofs], 
                                                self.goal_state[:,self.n_dofs:self.n_dofs*2], link_name='ee_link')
            self.goal_ee_quat = matrix_to_quaternion(self.goal_ee_rot)

            #normalize quaternion
            self.goal_ee_quat = self.goal_ee_quat / torch.norm(self.goal_ee_quat)

            self.curr_goal_ros.pose.position.x = self.goal_ee_pos[0][0]
            self.curr_goal_ros.pose.position.y = self.goal_ee_pos[0][1]
            self.curr_goal_ros.pose.position.z = self.goal_ee_pos[0][2]
            self.curr_goal_ros.pose.orientation.w = self.goal_ee_quat[0][0]
            self.curr_goal_ros.pose.orientation.x = self.goal_ee_quat[0][1]
            self.curr_goal_ros.pose.orientation.y = self.goal_ee_quat[0][2]
            self.curr_goal_ros.pose.orientation.z = self.goal_ee_quat[0][3]
            self.goal_is_valid = True
        else:
            self.goal_ee_pos = torch.zeros(1,3, **self.tensor_args)
            self.goal_ee_rot = torch.zeros(3,3, **self.tensor_args)
            self.goal_ee_quat = torch.zeros(1,4, **self.tensor_args)
            self.goal_is_valid = False

        self.curr_robot_state = JointState()
        self.curr_ee_pose_ros = PoseStamped()
        self.curr_ee_pose_ros.header = std_msgs.msg.Header()
        self.curr_ee_pos = None
        self.curr_ee_rot = None
        self.curr_ee_quat = None

        self.pose_cost_fn = self.setup_cost()
        
        if self.mode == 'interactive_marker':
            self.setup_interactive_marker_server()
        else:
            self.marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=1)
            self.setup_goal_marker()

        self.tstep = 0
        self.last_update_tstep = 0
        self.start_t = 0
        self.state_received = False
        self.init = True

        self.goal_dist_err_list = []
        self.goal_rot_err_list = []


    
    def mpc_command_callback(self, msg):
        self.init = False

    def marker_callback(self, msg):
        self.curr_goal_ros.header = msg.header
        self.curr_goal_ros.pose.position = msg.pose.position


        self.curr_goal_ros.pose.orientation.x = self.goal_ee_quat[0][0] 
        self.curr_goal_ros.pose.orientation.y = self.goal_ee_quat[0][1]
        self.curr_goal_ros.pose.orientation.z = self.goal_ee_quat[0][2]
        self.curr_goal_ros.pose.orientation.w = self.goal_ee_quat[0][3]

        # self.curr_goal_ros.pose.orientation.x = rot_const_goal_quat[0]
        # self.curr_goal_ros.pose.orientation.y = rot_const_goal_quat[1]
        # self.curr_goal_ros.pose.orientation.z = rot_const_goal_quat[2]
        # self.curr_goal_ros.pose.orientation.w = rot_const_goal_quat[3]


        self.goal_ee_pos[0][0] = self.curr_goal_ros.pose.position.x
        self.goal_ee_pos[0][1] = self.curr_goal_ros.pose.position.y
        self.goal_ee_pos[0][2] = self.curr_goal_ros.pose.position.z
        
    
    def goal_transform_callback(self, msg):
        if msg.pose.position.x != -5000 and \
           msg.pose.position.y != -5000 and \
           msg.pose.position.z != -5000:

            self.curr_goal_ros.header = msg.header
            self.curr_goal_ros.pose = msg.pose
            self.curr_goal_ros.pose.position.z += 0.01

            self.curr_goal_ros.pose.orientation.w = self.goal_ee_quat[0][0]
            self.curr_goal_ros.pose.orientation.x = self.goal_ee_quat[0][1]
            self.curr_goal_ros.pose.orientation.y = self.goal_ee_quat[0][2]
            self.curr_goal_ros.pose.orientation.z = self.goal_ee_quat[0][3]


            self.goal_ee_pos[0][0] = self.curr_goal_ros.pose.position.x
            self.goal_ee_pos[0][1] = self.curr_goal_ros.pose.position.y
            self.goal_ee_pos[0][2] = self.curr_goal_ros.pose.position.z

            # self.goal_ee_quat[0][0] = self.curr_goal_ros.pose.orientation.w
            # self.goal_ee_quat[0][1] = self.curr_goal_ros.pose.orientation.x
            # self.goal_ee_quat[0][2] = self.curr_goal_ros.pose.orientation.y
            # self.goal_ee_quat[0][3] = self.curr_goal_ros.pose.orientation.z

            self.goal_ee_rot = quaternion_to_matrix(self.goal_ee_quat)  
            self.goal_is_valid = True  
        else:
            self.goal_is_valid = False
        # print(self.curr_goal_ros.pose.position.x, self.curr_goal_ros.pose.position.y)
        # self.curr_goal_ros.pose.position.x = msg.transform.translation.x 
        # self.curr_goal_ros.pose.position.y = msg.transform.translation.y
        # self.curr_goal_ros.pose.position.z = msg.transform.translation.z

        # self.curr_goal_ros.pose.orientation = msg.transform.orientation


    def state_callback(self, msg):
        self.curr_robot_state = msg
        self.state_received = True

    def publish_goal_loop(self):
        while not rospy.is_shutdown():
        
            if (self.goal_ee_pos is not None) and \
               (self.goal_ee_rot is not None) and \
                self.goal_is_valid:

                if self.state_received:
                    self.update_ee_pose()
                    self.state_received = False
                    self.log_goal_error()
                # if self.tstep == 0 or ((self.last_check_tstep - self.tstep) >= 0.01):
                if self.mode == 'ee_goal_seq' or self.mode == 'ee_pose_seq':
                    if not self.init:
                        if (self.tstep - self.last_update_tstep >= 12.0):
                        # if self.goal_reached():
                            self.update_goal_from_sequence()
                            print(self.tstep, self.last_update_tstep)
                            rospy.loginfo('[Goal Manager]: Goal Updated')
                            self.last_update_tstep = self.tstep 
                    else:
                        self.last_update_tstep = self.tstep

                # rospy.loginfo('[Goal Manager] Curr Goal:', self.curr_goal_ros)
                self.curr_goal_ros.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.curr_goal_ros)

                if mode != "interactive_marker":
                    self.goal_marker.pose = self.curr_goal_ros.pose
                    # self.goal_marker.orientation = orientation
                    self.goal_marker.header.stamp = self.curr_goal_ros.header.stamp
                    self.marker_pub.publish(self.goal_marker) 

                    self.goal_transform.header.stamp = self.curr_goal_ros.header.stamp
                    self.goal_transform.transform.translation.x = self.curr_goal_ros.pose.position.x
                    self.goal_transform.transform.translation.y = self.curr_goal_ros.pose.position.y
                    self.goal_transform.transform.translation.z = self.curr_goal_ros.pose.position.z
                    self.goal_transform.transform.rotation = self.curr_goal_ros.pose.orientation
                    self.broadcaster.sendTransform(self.goal_transform)


                if self.tstep == 0:
                    self.start_t = rospy.get_time()

                self.tstep = rospy.get_time() - self.start_t               
                self.rate.sleep()
            else:
                rospy.logwarn('[GoalManager]: No valid goal')
                pass
        self.save_logged_data()


    def update_ee_pose(self):
        q = torch.as_tensor(self.curr_robot_state.position, **self.tensor_args).unsqueeze(0)
        qd = torch.as_tensor(self.curr_robot_state.velocity, **self.tensor_args).unsqueeze(0)

        self.curr_ee_pos, self.curr_ee_rot = self.robot_model.compute_forward_kinematics(q, qd, link_name='ee_link')
        self.curr_ee_quat = matrix_to_quaternion(self.curr_ee_rot)
        #normalize quaternion
        self.curr_ee_quat = self.curr_ee_quat / torch.norm(self.curr_ee_quat)

        #convert to pose stamped message and publish
        self.curr_ee_pose_ros.header.stamp = rospy.Time.now()
        self.curr_ee_pose_ros.pose.position.x = self.curr_ee_pos[0][0] 
        self.curr_ee_pose_ros.pose.position.y = self.curr_ee_pos[0][1] 
        self.curr_ee_pose_ros.pose.position.z = self.curr_ee_pos[0][2] 
        self.curr_ee_pose_ros.pose.orientation.w = self.curr_ee_quat[0][0] 
        self.curr_ee_pose_ros.pose.orientation.x = self.curr_ee_quat[0][1] 
        self.curr_ee_pose_ros.pose.orientation.y = self.curr_ee_quat[0][2] 
        self.curr_ee_pose_ros.pose.orientation.z = self.curr_ee_quat[0][3] 
        self.ee_pub.publish(self.curr_ee_pose_ros)


    def log_goal_error(self):
        if (self.curr_ee_pos is not None) and (self.curr_ee_rot is not None) and \
            (self.goal_ee_pos is not None) and (self.goal_ee_rot is not None):
            
            cost, rot_err_norm, goal_dist = self.pose_cost_fn.forward(self.curr_ee_pos, self.curr_ee_rot, self.goal_ee_pos, self.goal_ee_rot)       
            self.goal_dist_err_list.append(goal_dist)
            self.goal_rot_err_list.append(rot_err_norm)

    def save_logged_data(self):

        np.savez('/home/mohak/catkin_ws/src/franka_motion_control/data/mpc_error_data.npz', 'wb', 
                goal_dist_err = self.goal_dist_err_list, 
                goal_rot_err = self.goal_rot_err_list)
        print('[Goal Manager]: Logs dumped')


    def goal_reached(self):
        #get translation and rotation error
        reached = False
        if (self.curr_ee_pos is not None) and (self.curr_ee_rot is not None) and \
            (self.goal_ee_pos is not None) and (self.goal_ee_rot is not None):
            
            cost, rot_err_norm, goal_dist = self.pose_cost_fn.forward(self.curr_ee_pos, self.curr_ee_rot, self.goal_ee_pos, self.goal_ee_rot)
            print('goal dist', goal_dist, 'rot_err', rot_err_norm)
            if self.mode == 'ee_goal_seq':
                print(goal_dist)
                if goal_dist <= self.goal_pos_thresh:
                    reached = True
            elif  self.mode == 'ee_pose_seq':
                if goal_dist <= self.goal_pos_thresh and rot_err_norm <= self.goal_rot_thresh:
                    reached = True
        return reached


    def update_goal_from_sequence(self):

        self.curr_goal_idx = (self.curr_goal_idx + 1) % self.num_goals
        self.goal_state = torch.as_tensor(q_des_list[self.curr_goal_idx], **self.tensor_args).unsqueeze(0)

        self.goal_ee_pos, self.goal_ee_rot = self.robot_model.compute_forward_kinematics(self.goal_state[:,0:self.n_dofs], 
                                            self.goal_state[:,self.n_dofs:self.n_dofs*2], link_name='ee_link')
        self.goal_ee_quat = matrix_to_quaternion(self.goal_ee_rot)
        #normalize quaternion
        self.goal_ee_quat = self.goal_ee_quat / torch.norm(self.goal_ee_quat)


        self.curr_goal_ros.pose.position.x = self.goal_ee_pos[0][0]
        self.curr_goal_ros.pose.position.y = self.goal_ee_pos[0][1]
        self.curr_goal_ros.pose.position.z = self.goal_ee_pos[0][2]
        #Note: diff robot model uses wxyz convention for quaternion
        self.curr_goal_ros.pose.orientation.w = self.goal_ee_quat[0][0]
        self.curr_goal_ros.pose.orientation.x = self.goal_ee_quat[0][1]
        self.curr_goal_ros.pose.orientation.y = self.goal_ee_quat[0][2]
        self.curr_goal_ros.pose.orientation.z = self.goal_ee_quat[0][3]



    def setup_cost(self):

        vec_weight = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        weight = [1.0,1.0]
        position_gaussian_params = {'n':0, 'c':0.2, 's':0.0, 'r':10.0}
        orientation_gaussian_params =  {'n':0, 'c':0.2, 's':0.0, 'r':10.0}
        hinge_val =  -1
        convergence_val =  [0.0, 0.0] # orientation, position

        return PoseCost(weight, vec_weight, position_gaussian_params, 
                       orientation_gaussian_params, self.tensor_args, 
                       hinge_val, convergence_val)

    def setup_interactive_marker_server(self):

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("goal_marker")
        
        # create an interactive marker for our server
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "base_link"
        self.int_marker.name = "goal_marker"
        self.int_marker.description = "End-effector Goal"
        self.int_marker.scale = 0.1
        self.int_marker.pose.position = self.curr_goal_ros.pose.position


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

    def setup_goal_marker(self):
        self.goal_marker = Marker() 
        self.goal_marker.header.frame_id = 'base_link'
        self.goal_marker.type = 2
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 1.0
        self.goal_marker.color.b = 0.0
        self.goal_marker.color.a = 1.0
        self.goal_marker.scale.x = 0.05
        self.goal_marker.scale.y = 0.05
        self.goal_marker.scale.z = 0.05

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.goal_transform = geometry_msgs.msg.TransformStamped()

        # self.goal_transform.header.stamp = rospy.Time.now()
        self.goal_transform.header.frame_id = "base_link"
        self.goal_transform.child_frame_id = "ee_goal"


if __name__ == '__main__':
    rospy.init_node("goal_manager", anonymous=True)

    # mpc_yml_file = join_path(mpc_configs_path(), robot_params['mpc_yml'])
    joint_states_topic = rospy.get_param('~joint_states_topic')
    ee_goal_topic = rospy.get_param('~ee_goal_topic')
    ee_pose_topic = rospy.get_param('~ee_pose_topic')
    goal_sub_topic = rospy.get_param('~goal_sub_topic')
    publish_freq = rospy.get_param('~goal_pub_freq')
    urdf_path = rospy.get_param('~urdf_path')
    mode = rospy.get_param('~operation_mode')
    goal_pos_thresh = rospy.get_param('~goal_pos_thresh')
    goal_rot_thresh = rospy.get_param('~goal_rot_thresh')
 
    goal_manager = GoalManager(joint_states_topic,
                               ee_goal_topic,
                               ee_pose_topic,
                               urdf_path,
                               home_config,
                               goal_sub_topic,
                               publish_freq,
                               mode,
                               goal_pos_thresh,
                               goal_rot_thresh)
    
    rospy.loginfo('Initiating Goal Publisher')

    goal_manager.publish_goal_loop()