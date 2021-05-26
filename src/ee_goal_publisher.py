
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import std_msgs.msg
from visualization_msgs.msg import *

from storm_kit.differentiable_robot_model import DifferentiableRobotModel
from storm_kit.differentiable_robot_model.coordinate_transform import matrix_to_quaternion, quaternion_to_matrix
from storm_kit.mpc.cost import PoseCost

import numpy as np
import torch
import tf2_ros

goal_ee_pos tensor([[ 3.8689e-01,  4.4958e-01,  5.9739e-01],
        [ 4.7659e-01,  4.9783e-01,  2.8616e-01],
        [ 5.7576e-01, -4.7868e-01,  5.7678e-01],
        [ 5.4563e-01, -4.2102e-01,  2.8616e-01],
        [ 4.7659e-01,  4.9783e-01,  2.8616e-01],
        [ 3.0835e-01,  5.2710e-08,  4.9148e-01]], dtype=torch.float32)
goal_ee_quat tensor([[ 0.4595,  0.1146, -0.2016, -0.8573],
        [ 0.1133, -0.0171,  0.9911,  0.0680],
        [ 0.3932,  0.3132,  0.7399,  0.4470],
        [ 0.1254,  0.6598,  0.7397,  0.0417],
        [ 0.1133, -0.0171,  0.9911,  0.0680],
        [ 0.0000,  0.7055,  0.7087,  0.0000]], dtype=torch.float32)


class GoalManager(object):
    def __init__(self, fixed_frame="panda_link0",
                 max_goal_time=15.0):
        
        self.state_sub_topic = state_sub_topic
        self.goal_pub_topic = goal_pub_topic
        self.ee_pose_pub_topic = ee_pose_pub_topic
        self.goal_sub_topic = goal_sub_topic
        self.robot_urdf = robot_urdf 
        self.publish_freq = publish_freq
        self.operation_mode = operation_mode
        self.goal_pos_thresh = goal_pos_thresh
        self.goal_rot_thresh = goal_rot_thresh
        self.fixed_frame = fixed_frame
        self.ee_frame = ee_frame
        self.n_dofs = 7
        self.max_goal_time = max_goal_time


        #Setup ROS
        self.goal_pub = rospy.Publisher(self.goal_pub_topic, PoseStamped, queue_size=1, latch=False)
        self.state_sub = rospy.Subscriber(self.state_sub_topic, JointState, self.state_callback)

        self.rate = rospy.Rate(self.publish_freq)

        self.curr_goal_ros = PoseStamped()
        self.curr_goal_ros.header = std_msgs.msg.Header()
        self.curr_goal_ros.header.stamp = rospy.Time.now()

        self.state_received = False
        self.curr_robot_state = JointState()
        self.curr_ee_pose_ros = PoseStamped()
        self.curr_ee_pose_ros.header = std_msgs.msg.Header()
        self.curr_ee_pos = None
        self.curr_ee_rot = None
        self.curr_ee_quat = None

        
        #Set initial goal state
            # self. = len(q_des_list)
            self.num_goals = len(self.q_des_list)
            self.curr_goal_idx = 0
            self.goal_state = torch.as_tensor(self.q_des_list[self.curr_goal_idx], **self.tensor_args).unsqueeze(0)

            q_des_list_torch = torch.tensor(self.q_des_list)

            print('goal_ee_pos', goal_ee_pos)
            print('goal_ee_quat', goal_ee_quat)


        self.goal_ee_pos, self.goal_ee_rot = self.robot_model.compute_forward_kinematics(self.goal_state[:,0:self.n_dofs], 
                                            self.goal_state[:,self.n_dofs:self.n_dofs*2], link_name=self.ee_frame)
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





        self.pose_cost_fn = self.setup_cost()
        self.tstep = 0
        self.last_update_tstep = 0
        self.start_t = 0
        self.init = True

        self.goal_dist_err_list = []
        self.goal_rot_err_list = []


    def state_callback(self, msg):
        self.curr_robot_state = msg
        self.state_received = True

    def publish_goal_loop(self):
        while not rospy.is_shutdown():

            if (self.goal_ee_pos is not None) and \
               (self.goal_ee_rot is not None) and \
                self.goal_is_valid:
     
                if self.operation_mode == 'ee_goal_seq' or self.operation_mode == 'ee_pose_seq':
                    # if not self.init:
                    if (self.tstep - self.last_update_tstep >= self.max_goal_time):
                    # if self.goal_reached():
                        self.update_goal_from_sequence()
                        print(self.tstep, self.last_update_tstep)
                        rospy.loginfo('[Goal Manager]: Goal Updated')
                        self.last_update_tstep = self.tstep 
                    # else:
                    #     self.last_update_tstep = self.tstep
                    #     self.init = False

                # rospy.loginfo('[Goal Manager] Curr Goal:', self.curr_goal_ros)
                self.curr_goal_ros.header.stamp = rospy.Time.now()
                self.goal_pub.publish(self.curr_goal_ros)

                if self.operation_mode != "interactive_marker":
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
        # self.save_logged_data()


    def update_ee_pose(self):
        q = torch.as_tensor(self.curr_robot_state.position, **self.tensor_args).unsqueeze(0)
        qd = torch.as_tensor(self.curr_robot_state.velocity, **self.tensor_args).unsqueeze(0)

        self.curr_ee_pos, self.curr_ee_rot = self.robot_model.compute_forward_kinematics(q, qd, link_name=self.ee_frame)
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
            if self.operation_mode == 'ee_goal_seq':
                print(goal_dist)
                if goal_dist <= self.goal_pos_thresh:
                    reached = True
            elif  self.operation_mode == 'ee_pose_seq':
                if goal_dist <= self.goal_pos_thresh and rot_err_norm <= self.goal_rot_thresh:
                    reached = True
        return reached


    def update_goal_from_sequence(self):

        self.curr_goal_idx = (self.curr_goal_idx + 1) % self.num_goals
        self.goal_state = torch.as_tensor(self.q_des_list[self.curr_goal_idx], **self.tensor_args).unsqueeze(0)

        self.goal_ee_pos, self.goal_ee_rot = self.robot_model.compute_forward_kinematics(self.goal_state[:,0:self.n_dofs], 
                                            self.goal_state[:,self.n_dofs:self.n_dofs*2], link_name=self.ee_frame)
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
        self.int_marker.header.frame_id = self.fixed_frame #"panda_link0"
        self.int_marker.name = "goal_marker"
        self.int_marker.description = "End-effector Goal"
        self.int_marker.scale = 0.2
        # self.int_marker.pose.position = self.curr_goal_ros.pose.position
        self.int_marker.pose = self.curr_goal_ros.pose


        # create a grey box marker
        self.box_marker = Marker()
        self.box_marker.type = Marker.CUBE
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

        # create controls for rotation
        self.rotate_x_control = InteractiveMarkerControl()
        self.rotate_x_control.orientation.w = 1
        self.rotate_x_control.orientation.x = 1
        self.rotate_x_control.orientation.y = 0
        self.rotate_x_control.orientation.z = 0
        self.rotate_x_control.name = "rotate_x"
        self.rotate_x_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # if fixed:
            # control.orientation_mode = InteractiveMarkerControl.FIXED
        self.int_marker.controls.append(self.rotate_x_control)

        self.rotate_y_control = InteractiveMarkerControl()
        self.rotate_y_control.orientation.w = 1
        self.rotate_y_control.orientation.x = 0
        self.rotate_y_control.orientation.y = 1
        self.rotate_y_control.orientation.z = 0
        self.rotate_y_control.name = "rotate_y"
        self.rotate_y_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # if fixed:
            # control.orientation_mode = InteractiveMarkerControl.FIXED
        self.int_marker.controls.append(self.rotate_y_control)

        self.rotate_z_control = InteractiveMarkerControl()
        self.rotate_z_control.orientation.w = 1
        self.rotate_z_control.orientation.x = 0
        self.rotate_z_control.orientation.y = 0
        self.rotate_z_control.orientation.z = 1
        self.rotate_z_control.name = "rotate_z"
        self.rotate_z_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # if fixed:
            # control.orientation_mode = InteractiveMarkerControl.FIXED
        self.int_marker.controls.append(self.rotate_z_control)

        # add the interactive marker to our collection &
        # tell the server to call marker_callback() when feedback arrives for it
        self.server.insert(self.int_marker, self.marker_callback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def setup_goal_marker(self):
        self.goal_marker = Marker() 
        self.goal_marker.header.frame_id = self.fixed_frame #'panda_link0'
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
        self.goal_transform.header.frame_id = "panda_link0"
        self.goal_transform.child_frame_id = "ee_goal"
    
    def close(self):
        pass


if __name__ == '__main__':
    rospy.init_node("goal_manager", anonymous=True, disable_signals=True)

    # mpc_yml_file = join_path(mpc_configs_path(), robot_params['mpc_yml'])
    joint_states_topic = rospy.get_param('~joint_states_topic')
    ee_goal_topic = rospy.get_param('~ee_goal_topic')
    ee_pose_topic = rospy.get_param('~ee_pose_topic')
    goal_sub_topic = rospy.get_param('~goal_sub_topic')
    publish_freq = rospy.get_param('~goal_pub_freq')
    robot_urdf = rospy.get_param('~robot_urdf')
    operation_mode = rospy.get_param('~operation_mode')
    goal_pos_thresh = rospy.get_param('~goal_pos_thresh')
    goal_rot_thresh = rospy.get_param('~goal_rot_thresh')
    fixed_frame = rospy.get_param('~fixed_frame')
    ee_frame = rospy.get_param('~ee_frame')
    max_goal_time = rospy.get_param('~max_goal_time')
 
    goal_manager = GoalManager(joint_states_topic,
                               ee_goal_topic,
                               ee_pose_topic,
                               robot_urdf,
                               goal_sub_topic,
                               publish_freq,
                               operation_mode,
                               goal_pos_thresh,
                               goal_rot_thresh,
                               fixed_frame,
                               ee_frame)
    
    rospy.loginfo('Initiating Goal Publisher')

    try:
        goal_manager.publish_goal_loop()
    except KeyboardInterrupt:
        goal_manager.close()
