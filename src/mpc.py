#! /usr/bin/env python
import os
import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
import ros_numpy
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
import torch
torch.multiprocessing.set_start_method('spawn',force=True)

import yaml

# from stochastic_control.mpc_tools.rollout.arm_reacher import ArmReacher
from stochastic_control.mpc_tools.rollout.arm_reacher_nn_collision import ArmReacherCollisionNN
from stochastic_control.mpc_tools.control import MPPI, StompMPPI
from stochastic_control.mpc_tools.utils.state_filter import JointStateFilter, RobotStateFilter
from stochastic_control.mpc_tools.utils.mpc_process_wrapper import ControlProcess
from stochastic_control.utils.util_file import get_configs_path, get_gym_configs_path, get_mpc_configs_path, join_path, load_yaml, get_assets_path
from cubic_spline import CubicSplineInterPolation

from stochastic_control_conversions import *
from differentiable_robot_model.coordinate_transform import quaternion_to_matrix, CoordinateTransform
np.set_printoptions(precision=6)



class MPCController(object):
    def __init__(self, robot_state_topic, command_topic, goal_topic, pointcloud_topic,
                filtered_state_topic, config_file, control_freq, fixed_frame,  
                pointcloud_frame, debug=False, joint_names=[]):
        #user_command_topic
        self.robot_state_topic = robot_state_topic
        self.command_topic = command_topic
        self.goal_topic = goal_topic
        self.pointcloud_topic = pointcloud_topic
        self.filtered_state_topic = filtered_state_topic
        # self.user_command_topic = user_command_topic
        self.config_file = config_file
        self.control_freq = control_freq
        self.fixed_frame = fixed_frame
        self.pointcloud_frame = pointcloud_frame
        self.debug = debug
        self.joint_names = joint_names

        # self.goal_state_list = goal_state_list
        # self.curr_goal_idx = 0
        # self.curr_goal = self.goal_state_list[self.curr_goal_idx]

        # #Initialize MPC controller
        self.sim_dt = 1.0 / self.control_freq #0.001
        self.initialize_mpc_controller()
        # self.control_process.update_goal(goal_state=self.curr_goal)
        # self.controller.rollout_fn.update_goal(goal_state=self.curr_goal)

        #Create filters for state and command
        # self.robot_command_filter = JointStateFilter(filter_coeff={'position': 0.01, 'velocity': 0.01})                                                     
        # # self.robot_state_filter = JointStateFilter(filter_coeff={'position': 1.0, 'velocity': 0.1}, dt=0.0)
        # self.robot_state_filter = JointStateFilter(filter_coeff={'position': 1.0, 'velocity': 1.0}, dt=0.0)
        # self.state_filter_coeff = {'position': 0.1, 'velocity':0.01, 'acceleration':1.0}
        self.state_filter_coeff = {'position': 0.8, 'velocity': 0.5, 'acceleration':1.0}
        self.robot_state_filter = RobotStateFilter(filter_coeff=self.state_filter_coeff,
                                                   dt=self.exp_params['control_dt'],
                                                   filter_keys=self.state_filter_coeff.keys())
        # self.pos_filter_coeff = {'position': 0.8} # 'velocity': 1.0, 'acceleration':1.0}
        # self.position_state_filter = RobotStateFilter(filter_coeff=self.pos_filter_coeff,
        #                                               dt=self.exp_params['control_dt'],
        #                                               filter_keys=self.pos_filter_coeff.keys())
        # self.vel_filter_coeff = {'position': 0.8} # 'velocity': 1.0, 'acceleration':1.0}
        # self.vel_state_filter = RobotStateFilter(filter_coeff=self.vel_filter_coeff,
        #                                               dt=self.exp_params['control_dt'],
        #                                               filter_keys=self.vel_filter_coeff.keys())


        self.command_filter_coeff = {'position': 0.1, 'velocity': 0.01, 'acceleration': 1.0}
        self.robot_command_filter = JointStateFilter(filter_coeff=self.command_filter_coeff, 
                                                     dt=self.exp_params['control_dt'],
                                                     filter_keys=self.command_filter_coeff.keys())
        
        self.spline = CubicSplineInterPolation()
        rospy.loginfo('[MPC]: Controller Initialized')

        #Initialize ROS
        self.command_pub = rospy.Publisher(self.command_topic, JointState, queue_size=1, latch=False)
        self.filtered_state_pub = rospy.Publisher(self.filtered_state_topic, JointState, queue_size=1, latch=False)

        self.state_sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        # self.user_command_sub = rospy.Subscriber(self.user_command_topic, String, self.user_command_callback)
        self.rate = rospy.Rate(self.control_freq)

        #Variables for book-keeping
        self.curr_state_raw = None
        self.curr_state_filtered = None
        self.curr_state_raw_dict = {}
        self.prev_state_filtered_dict = None
        self.curr_state_filtered_dict = {}
        self.curr_ee_goal = None
        self.curr_ee_goal_pos = None
        self.curr_ee_goal_quat = None
        self.curr_pointcloud = None
        self.curr_mpc_command = JointState()
        self.curr_mpc_command.name = self.joint_names
        self.stop_controller = False
        self.state_received_flag = False
        self.zero_acc = np.zeros(7)
        self.prev_mpc_qdd_des = np.zeros(7)
        self.tstep = 0
        self.prev_tstep = 0
        self.last_command_tstep = 0
        self.pointcloud_received = False
        if self.debug:
            self.tsteps = []
            self.robot_q_list_r = []
            self.robot_qd_list_r = []
            self.robot_qd_des_list_r = []
            self.robot_q_list_f = []
            self.robot_qd_list_f = []
            self.robot_qd_des_list_f = []
            self.command_q_list = []
            self.command_qd_list = []
            self.command_qdd_list = []
            self.ee_goal_pos_list = []
            self.ee_goal_quat_list = []

    def state_callback(self, msg):
        # self.prev_state_raw = self.curr_state_raw
        self.curr_state_raw = msg
        self.curr_state_raw_dict = joint_state_to_dict(self.curr_state_raw)
        self.state_received_flag = True
        
        # #truncate the velocities
        # self.curr_state_raw.velocity = np.where(np.abs(self.curr_state_raw.velocity) > 0.05, 
        #                                         self.curr_state_raw.velocity, 0.0)
        #filter states

        #filter velocity only
        # self.curr_state_filtered_dict = self.curr_state_raw_dict
        # self.curr_state_filtered_dict['velocity'] = self.robot_state_filter.filter_joint_state({'velocity': self.curr_state_raw_dict['velocity']})['velocity']
                    # filtered_state = robot_state_filter.filter_state(current_state, sim_dt)
        # self.curr_state_filtered_dict = self.robot_state_filter.filter_state(self.curr_state_raw_dict, 0.001)
        # self.curr_state_filtered = dict_to_joint_state(self.curr_state_filtered_dict)


    def pointcloud_callback(self, msg):
        self.curr_pointcloud = msg
        if not self.pointcloud_received:
            self.pointcloud_received = True
            data = pointcloud2_to_np(self.curr_pointcloud)
            np.savez('/home/mohak/catkin_ws/src/franka_motion_control/data/pointcloud.npz', 'wb',
                    data=data)
            self.controller.rollout_fn.voxel_collision_cost.coll.set_scene(data, np.zeros_like(data))

    def goal_callback(self, msg):
        #Update mpc goal
        self.curr_ee_goal = msg
        self.curr_ee_goal_pos, self.curr_ee_goal_quat = pose_stamped_to_np(msg)
        self.control_process.update_goal(goal_ee_pos = self.curr_ee_goal_pos,
                                         goal_ee_quat = self.curr_ee_goal_quat)
        self.controller.rollout_fn.update_goal(goal_ee_pos = self.curr_ee_goal_pos,
                                               goal_ee_quat = self.curr_ee_goal_quat)
    
    def control_loop_spline(self):
        
        while not rospy.is_shutdown():
        
            if self.curr_state_raw is not None: # and self.curr_ee_goal is not None:
                self.curr_state_filtered_dict = self.robot_state_filter.filter_state(self.curr_state_raw_dict, self.sim_dt)
                curr_state_np = np.hstack((self.curr_state_filtered_dict['position'], 
                                           self.curr_state_filtered_dict['velocity'], 
                                           self.curr_state_filtered_dict['acceleration']))
                # curr_state_np = np.concatenate((self.curr_state_filtered_dict['position'], 
                #                                 self.curr_state_filtered_dict['velocity'], 
                #                                 self.prev_mpc_qdd_des), axis=0)
                curr_state_tensor = torch.as_tensor(curr_state_np)

                mpc_next_command, command_dt, val, info = self.control_process.get_command(self.tstep, curr_state_tensor,
                                                                                           debug=False,
                                                                                           control_dt=self.sim_dt)
                mpc_qdd_des = np.ravel(mpc_next_command[0]) 
                mpc_cmd_dt = command_dt[0].item()

                mpc_cmd_des = self.robot_command_filter.integrate_acc(mpc_qdd_des, 
                                        self.curr_state_filtered_dict,
                                        dt = mpc_cmd_dt - self.tstep)


                
                #calculate new spline command if unavailable or mpc command updated
                if (not self.spline.command_available(self.tstep)) or \
                   (not (mpc_qdd_des == self.prev_mpc_qdd_des).all()): 
                    # print('Fitting spline', mpc_qdd_des, self.prev_mpc_qdd_des, self.tstep, mpc_cmd_dt, self.control_process.mpc_dt, self.spline.command_available(self.tstep))                                                                           
                    # if(self.exp_params['control_space'] == 'acc'):
                    # qdd_des = np.ravel(next_command[0])
                    # curr_command_dt = command_dt[0].item()
                    # print('Curr command dt', curr_command_dt, self.tstep)

                    mpc_des_state_torch = torch.cat((torch.tensor([mpc_cmd_des['position']]),
                                                    torch.tensor([mpc_cmd_des['velocity']])),
                                                    dim=0)
                    # curr_state_spline = torch.as_tensor(curr_state_np[0:14].reshape(2,7))      
                    curr_state_spline = curr_state_tensor[0:14].reshape(2,7)
                    #fit spline
                    #TBD: Question: Should fitting be donee from curr state of robot or current spline state?
                    self.spline.fit(curr_state_spline, self.tstep, mpc_des_state_torch, mpc_cmd_dt)      



                spline_cmd_des = self.spline.get_command(self.tstep)
                self.curr_mpc_command.position = spline_cmd_des[0].detach().numpy()
                self.curr_mpc_command.velocity = np.zeros(7) #spline_cmd_des[1] #return spline derivative as well   
                self.curr_mpc_command.effort = np.zeros(7) #What is the third key here??

                self.command_pub.publish(self.curr_mpc_command)
                # rospy.loginfo('[MPC]: Command published')
                
                if self.tstep == 0:
                    self.start_t = rospy.get_time()
                    # self.start_t = time.time()

                # self.tstep = time.time() - self.start_t 
                self.tstep = rospy.get_time() - self.start_t
                self.prev_mpc_qdd_des = mpc_qdd_des


                # print(self.tstep, time.time(), rospy.get_time(), rospy.get_time()-self.start_t_ros)
                # ee_error,_ = self.controller.rollout_fn.current_cost(curr_state_tensor.unsqueeze(0))
                # ee_error = [x.detach().cpu().item() for x in ee_error]
                # rospy.loginfo(["{:.3f}".format(x) for x in ee_error]) #, "{:.3f}".format(mpc_control.mpc_dt)
                
                if self.debug:
                    self.log_data(mpc_qdd_des, [0., 0., 0.])

            elif self.curr_state_raw is None:
                rospy.loginfo('[MPC]: Waiting for state')
            
            elif self.curr_ee_goal is None:
                rospy.loginfo('[MPC]: Waiting for goal`')

            self.rate.sleep()

        self.close()
    

    def control_loop_linear(self):

        while not rospy.is_shutdown():
        
            if self.curr_state_raw is not None and self.curr_ee_goal is not None: # and self.curr_pointcloud is not None:

                if self.state_received_flag:     
                    self.curr_state_filtered_dict = self.robot_state_filter.filter_state(self.curr_state_raw_dict, 0.01) #filters position  
                    # if self.prev_state_filtered_dict is not None:
                        # self.curr_state_filtered_dict['velocity'] = (self.curr_state_filtered_dict['position'] - self.prev_state_filtered_dict['position']) / 0.01           
                        # self.curr_state_filtered_dict['velocity'] = self.velocity_state_filter.filter_state(self.curr_state_filtered_dict, 0.01)['velocity'] 
                        # self.curr_state_filtered_dict['acceleration'] = (self.curr_state_filtered_dict['velocity'] - self.prev_state_filtered_dict['velocity']) / 0.01
                    self.curr_state_filtered = dict_to_joint_state(self.curr_state_filtered_dict)
                    self.state_received_flag = False
                
                
                curr_state_np = np.hstack((self.curr_state_filtered_dict['position'], 
                                           self.curr_state_filtered_dict['velocity'], 
                                           self.curr_state_filtered_dict['acceleration']))
                
                # curr_state_tensor = torch.as_tensor(curr_state_np, **self.mpc_tensor_dtype) #.unsqueeze(0)

                if (self.tstep == 0) or (self.tstep - self.last_command_tstep) >= 0.01: # \
                    # or (not self.spline.command_available(self.tstep)):
                    mpc_next_command, mpc_command_tstep, val, info = self.control_process.get_command(self.tstep, curr_state_np,
                                                                                                     debug=False,
                                                                                                     control_dt=0.01) 
                    if(self.exp_params['control_space'] == 'acc'):
                        command = self.robot_command_filter.integrate_acc(mpc_next_command, self.curr_state_filtered_dict, dt=0.01)
                    elif(self.exp_params['control_space'] == 'vel'):
                        command = self.robot_command_filter.integrate_vel(mpc_next_command, self.curr_state_filtered_dict, dt=0.01)
                    elif(self.exp_params['control_space'] == 'jerk'):
                        command = self.robot_command_filter.integrate_jerk(mpc_next_command, self.curr_state_filtered_dict, dt=0.01)
                 
                self.curr_mpc_command.position = command['position']
                self.curr_mpc_command.velocity = command['velocity']
                self.curr_mpc_command.effort = command['acceleration'] 
                
                self.command_pub.publish(self.curr_mpc_command) #publish mpc command
                self.filtered_state_pub.publish(self.curr_state_filtered) #publish filtered state

                rospy.loginfo('[MPC]: Command published')

                if self.tstep == 0:
                    self.start_t = rospy.get_time()

                self.prev_tstep = self.tstep
                self.tstep = rospy.get_time() - self.start_t
                self.prev_state_filtered_dict = self.curr_state_raw_dict
                                
                # self.prev_mpc_qdd_des = mpc_qdd_des
                
                # ee_error,_ = self.controller.rollout_fn.current_cost(curr_state_tensor.unsqueeze(0))
                # ee_error = [x.detach().cpu().item() for x in ee_error]
                # rospy.loginfo(["{:.3f}".format(x) for x in ee_error]) #, "{:.3f}".format(mpc_control.mpc_dt)
                
                if self.debug:
                    self.log_data(mpc_next_command)

            elif self.curr_state_raw is None:
                rospy.loginfo('[MPC]: Waiting for state')
            
            elif self.curr_ee_goal is None:
                rospy.loginfo('[MPC]: Waiting for ee goal')
            
            elif self.curr_pointcloud is None:
                rospy.loginfo('[MPC]: Waiting for pointcloud')

            self.rate.sleep()

        self.close()
    

    def initialize_mpc_controller(self):
        
        with open(self.config_file) as file:
            self.robot_params = yaml.load(file, Loader=yaml.FullLoader)


        self.mpc_yml_file = join_path(get_mpc_configs_path(), self.robot_params['mpc_yml'])
        with open(self.mpc_yml_file) as file:
            self.exp_params = yaml.load(file, Loader=yaml.FullLoader)

        float_dtype = torch.float32
        if(self.exp_params['float_dtype'] == 'float16'):
            float_dtype = torch.bfloat16
        use_cuda = torch.cuda.is_available() if self.exp_params['use_cuda'] else False
        device = torch.device('cuda', self.exp_params['cuda_device_num']) if use_cuda else torch.device('cpu')
        self.mpc_tensor_dtype = {'device':device, 'dtype':float_dtype}


        self.exp_params['robot_params'] = self.robot_params
        rollout_fn = ArmReacherCollisionNN(self.exp_params, device=device, float_dtype=float_dtype, world_params=None)

        #Initialize world for collision cost
        # temp_trans = torch.ones(3, device=device, dtype=float_dtype)
        # temp_rot = torch.ones(3,3, device=device, dtype=float_dtype)
        # robot_c_trans = torch.tensor([1.114, -0.402, 0.802], device=device, dtype=float_dtype)
        # robot_c_quat = torch.tensor([0.513, 0.698, -0.409, -0.288], device=device, dtype=float_dtype)
        # robot_c_rot = quaternion_to_matrix(robot_c_quat)
        # rollout_fn.voxel_collision_cost.coll.set_world_transform(temp_trans, temp_rot, robot_c_trans, robot_c_rot)
        # robot_table_trans, robot_R_table, robot_c_trans, robot_R_c

        mppi_params = self.exp_params['mppi']
        dynamics_model = rollout_fn.dynamics_model
        if(self.exp_params['control_space'] == 'pos'):
            self.exp_params['model']['max_acc'] = 3.0
        #Create controller
        mppi_params['d_action'] = dynamics_model.d_action
        mppi_params['action_lows'] = -self.exp_params['model']['max_acc'] * torch.ones(dynamics_model.d_action, device=device, dtype=float_dtype)
        mppi_params['action_highs'] = self.exp_params['model']['max_acc'] * torch.ones(dynamics_model.d_action, device=device, dtype=float_dtype)
        init_action = torch.zeros((mppi_params['horizon'], dynamics_model.d_action), device=device)
        # init_action[:,:] += init_q
        # if(self.exp_params['control_space'] == 'acc'):
            # mppi_params['init_mean'] = init_action * 0.0 # device=device)
        # elif(self.exp_params['control_space'] == 'pos'):
            # mppi_params['init_mean'] = init_action 
        mppi_params['init_mean'] = init_action 
        mppi_params['rollout_fn'] = rollout_fn
        mppi_params['device'] = device
        mppi_params['float_dtype'] = float_dtype

        self.controller = MPPI(**mppi_params)
        self.control_process = ControlProcess(self.controller, control_space=self.exp_params['control_space'], control_dt=self.sim_dt)
        self.controller.rollout_fn.dynamics_model.robot_model.load_lxml_objects()


    def log_data(self, qdd_des):
        self.robot_q_list_r.append(self.curr_state_raw.position)
        self.robot_qd_list_r.append(self.curr_state_raw.velocity)
        self.robot_qd_des_list_r.append(self.curr_state_raw.effort)
        
        self.robot_q_list_f.append(self.curr_state_filtered_dict['position'])
        self.robot_qd_list_f.append(self.curr_state_filtered_dict['velocity'])
        self.robot_qd_des_list_f.append(self.curr_state_filtered_dict['acceleration'])
        
        self.command_q_list.append(self.curr_mpc_command.position)
        self.command_qd_list.append(self.curr_mpc_command.velocity)
        self.command_qdd_list.append(qdd_des)

        # self.ee_dist_err_list.append(ee_error[0])
        # self.ee_rot_err_list.append(ee_error[1])

        self.ee_goal_pos_list.append(self.curr_ee_goal_pos)
        self.ee_goal_quat_list.append(self.curr_ee_goal_quat)

        self.tsteps.append(self.tstep)

    def close(self):
        print('Closing MPC Controller')
        if self.debug:
            np.savez('/home/mohak/catkin_ws/src/franka_motion_control/data/mpc_data.npz', 'wb', 
                    q_robot_r = self.robot_q_list_r, 
                    qd_robot_r = self.robot_qd_list_r,
                    qd_des_robot_r = self.robot_qd_des_list_r,
                    q_robot_f = self.robot_q_list_f, 
                    qd_robot_f = self.robot_qd_list_f,
                    qd_des_robot_f = self.robot_qd_des_list_f,
                    q_cmd = self.command_q_list,
                    qd_cmd = self.command_qd_list,
                    qdd_cmd = self.command_qdd_list,
                    ee_goal_pos = self.ee_goal_pos_list,
                    ee_goal_quat = self.ee_goal_quat_list,
                    tsteps = self.tsteps)
            print('Logs dumped')

    # def joint_state_to_dict(self, msg):
    #     return {'position': np.array(msg.position), 
    #             'velocity': np.array(msg.velocity)} 
    #             #'acceleration': np.array(msg.effort) 
    
    # def dict_to_joint_state(self, dict):
    #     msg = JointState()
    #     msg.position = dict['position']
    #     msg.velocity = dict['velocity']
    #     # msg.effort = dict['acceleration'] 
    #     return msg
    
    # def pose_stamped_to_np(self, msg):
    #     pos = np.array([msg.pose.position.x,
    #                     msg.pose.position.y,
    #                     msg.pose.position.z])
    #     quat = np.array([msg.pose.orientation.x,
    #                      msg.pose.orientation.y,
    #                      msg.pose.orientation.z,
    #                      msg.pose.orientation.w])
    #     return pos, quat
    
    # def pointcloud2_to_np(self, msg):
    #     pc = ros_numpy.numpify(msg)
    #     points=np.zeros((pc.shape[0],3))
    #     points[:,0]=pc['x']
    #     points[:,1]=pc['y']
    #     points[:,2]=pc['z']
    #     return np.array(points)


if __name__ == '__main__':
    rospy.init_node("mpc_controller", anonymous=True)

    # config_file = join_path(mpc_configs_path(), robot_params['mpc_yml'])
    config_file = os.path.abspath(rospy.get_param('~config_file'))
    joint_states_topic = rospy.get_param('~joint_states_topic')
    joint_command_topic = rospy.get_param('~joint_command_topic')
    ee_goal_topic = rospy.get_param('~ee_goal_topic')
    pointcloud_topic = rospy.get_param('~pointcloud_topic')
    filtered_state_topic = rospy.get_param('~filtered_state_topic')
    control_freq = rospy.get_param('~control_freq')
    fixed_frame = rospy.get_param('~fixed_frame')
    pointcloud_frame = rospy.get_param('~pointcloud_frame')
    debug = rospy.get_param('~debug')
    joint_names = rospy.get_param('~joint_names')

    mpc_controller = MPCController(joint_states_topic,
                                   joint_command_topic,
                                   ee_goal_topic,
                                   pointcloud_topic,
                                   filtered_state_topic,
                                   config_file,
                                   control_freq,
                                   fixed_frame,
                                   pointcloud_frame,
                                   debug,
                                   joint_names)

    rospy.loginfo('[MPC]: Initiating Control Loop')
    mpc_controller.control_loop_linear()