#! /usr/bin/env python
import os
import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import torch
torch.multiprocessing.set_start_method('spawn',force=True)

import yaml

# from stochastic_control.mpc_tools.rollout.arm_reacher import ArmReacher
from stochastic_control.mpc_tools.rollout.arm_reacher_nn_collision import ArmReacherCollisionNN
from stochastic_control.mpc_tools.control import MPPI, StompMPPI
from stochastic_control.mpc_tools.utils.state_filter import JointStateFilter
from stochastic_control.mpc_tools.utils.mpc_process_wrapper import ControlProcess
from stochastic_control.utils.util_file import get_configs_path, get_gym_configs_path, join_path, load_yaml, get_assets_path
from cubic_spline import CubicSplineInterPolation

from differentiable_robot_model.coordinate_transform import quaternion_to_matrix, CoordinateTransform
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




class MPCController(object):
    def __init__(self, robot_state_topic, command_topic, goal_topic, mpc_yml_file, goal_state_list, control_freq, debug=False, joint_names=[]):
        #user_command_topic
        self.robot_state_topic = robot_state_topic
        self.command_topic = command_topic
        self.goal_topic = goal_topic
        # self.user_command_topic = user_command_topic
        self.mpc_yml_file = mpc_yml_file
        self.control_freq = control_freq
        self.debug = debug
        self.joint_names = joint_names

        self.goal_state_list = goal_state_list
        self.curr_goal_idx = 0
        self.curr_goal = self.goal_state_list[self.curr_goal_idx]

        #Initialize MPC controller
        self.initialize_mpc_controller()
        self.control_process.update_goal(goal_state=self.curr_goal)
        self.controller.rollout_fn.update_goal(goal_state=self.curr_goal)

        #Create filters for state and command
        self.robot_command_filter = JointStateFilter(filter_coeff={'position': 0.01, 'velocity': 0.01})                                                     
        # self.robot_state_filter = JointStateFilter(filter_coeff={'position': 1.0, 'velocity': 0.1}, dt=0.0)
        self.robot_state_filter = JointStateFilter(filter_coeff={'position': 1.0, 'velocity': 1.0}, dt=0.0)

        self.spline = CubicSplineInterPolation()
        rospy.loginfo('[MPC]: Controller Initialized')

        #Initialize ROS
        self.pub = rospy.Publisher(self.command_topic, JointState, queue_size=1, latch=False)
        self.state_sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)
        # self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback)
        # self.user_command_sub = rospy.Subscriber(self.user_command_topic, String, self.user_command_callback)
        self.rate = rospy.Rate(self.control_freq)

        #Variables for book-keeping
        self.curr_state_raw = None
        self.curr_state_filtered = None
        self.curr_state_raw_dict = {}
        self.curr_state_filtered_dict = {}
        # self.curr_ee_goal = None
        self.curr_mpc_command = JointState()
        self.curr_mpc_command.name = self.joint_names
        self.stop_controller = False
        self.zero_acc = np.zeros(7)
        self.prev_mpc_qdd_des = np.zeros(7)
        self.tstep = 0
        if self.debug:
            self.robot_q_list_r = []
            self.robot_qd_list_r = []
            self.robot_qd_des_list_r = []
            self.robot_q_list_f = []
            self.robot_qd_list_f = []
            self.robot_qd_des_list_f = []
            self.command_q_list = []
            self.command_qdd_list = []
            self.ee_dist_err_list = []
            self.ee_rot_err_list = []

    # def user_command_callback(self, msg):
    #     if msg.data == "stop":
    #         self.stop_controller = True
    #         rospy.loginfo('Stop command received')

    def state_callback(self, msg):
        self.curr_state_raw = msg
        
        # #truncate the velocities
        # self.curr_state_raw.velocity = np.where(np.abs(self.curr_state_raw.velocity) > 0.05, 
        #                                         self.curr_state_raw.velocity, 0.0)
        #filter states
        self.curr_state_raw_dict = self.joint_state_to_dict(self.curr_state_raw)

        #filter velocity only
        # self.curr_state_filtered_dict = self.curr_state_raw_dict
        # self.curr_state_filtered_dict['velocity'] = self.robot_state_filter.filter_joint_state({'velocity': self.curr_state_raw_dict['velocity']})['velocity']
        
        self.curr_state_filtered_dict = self.robot_state_filter.filter_joint_state(self.curr_state_raw_dict)
        self.curr_state_filtered = self.dict_to_joint_state(self.curr_state_filtered_dict)


    # def goal_callback(self, msg):
        # self.curr_ee_goal = msg
        # pass
        # goal_ee_pos, goal_ee_quat = self.pose_stamped_to_np(msg)

        # self.control_process.update_goal(goal_ee_pos = goal_ee_pos,
        #                                  goal_ee_quat = goal_ee_quat)
        # self.controller.rollout_fn.update_goal(goal_ee_pos = goal_ee_pos,
        #                                        goal_ee_quat = goal_ee_quat)
    
    def control_loop_spline(self):
        
        while not rospy.is_shutdown():
        
            if self.curr_state_raw is not None: # and self.curr_ee_goal is not None:
                curr_state_np = np.hstack((self.curr_state_filtered_dict['position'], 
                                           self.curr_state_filtered_dict['velocity'], 
                                           self.prev_mpc_qdd_des))
                # curr_state_np = np.concatenate((self.curr_state_filtered_dict['position'], 
                #                                 self.curr_state_filtered_dict['velocity'], 
                #                                 self.prev_mpc_qdd_des), axis=0)
                curr_state_tensor = torch.as_tensor(curr_state_np)

                mpc_next_command, command_dt, val, info = self.control_process.get_command(self.tstep, curr_state_tensor,
                                                                                           debug=False,
                                                                                           control_dt=self.control_process.mpc_dt)
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
                    #TODO: Remove the joint filter and only use MPC command.

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

                self.pub.publish(self.curr_mpc_command)
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
        
            if self.curr_state_raw is not None: # and self.curr_ee_goal is not None:
                
                curr_state_np = np.hstack((self.curr_state_filtered_dict['position'], 
                                           self.curr_state_filtered_dict['velocity'], 
                                           self.prev_mpc_qdd_des))
                # curr_state_np = np.concatenate((self.curr_state_filtered_dict['position'], 
                #                                 self.curr_state_filtered_dict['velocity'], 
                #                                 self.prev_mpc_qdd_des), axis=1)
                curr_state_tensor = torch.as_tensor(curr_state_np)
                mpc_next_command, command_dt, val, info = self.control_process.get_command(self.tstep, curr_state_tensor,
                                                                                           debug=False,
                                                                                           control_dt=self.control_process.mpc_dt)
                mpc_qdd_des = np.ravel(mpc_next_command[0]) 
                mpc_cmd_dt = command_dt[0].item()

                # mpc_cmd_des = self.robot_command_filter.integrate_acc(mpc_qdd_des, 
                #                         self.curr_state_filtered_dict,
                #                         dt = mpc_cmd_dt - self.tstep)
                                                             
                # if(self.exp_params['control_space'] == 'acc'):
                #     qdd_des = np.ravel(next_command[0])
                #     curr_command_dt = command_dt[0].item()
                #     # print('Curr command dt', curr_command_dt, self.tstep)
                #     mpc_cmd_des = self.robot_command_filter.integrate_acc(qdd_des, 
                #                             self.curr_state_filtered_dict,
                #                             dt = curr_command_dt - self.tstep)
                #     self.curr_mpc_command.position = mpc_cmd_des['position']
                #     self.curr_mpc_command.velocity = mpc_cmd_des['velocity']
                #     self.curr_mpc_command.effort = np.zeros(7) #What is the third key here??
                #     self.prev_mpc_qdd_des = qdd_des
                    
                # elif(self.exp_params['control_space'] == 'pos'):
                #     self.curr_mpc_command.position = np.ravel(next_command[0])
                #     self.curr_mpc_command.velocity = np.zeros(7)
                #     self.curr_mpc_command.effort = np.zeros(7)
                    
                    # mpc_des_state_np = np.concatenate(([mpc_cmd_des['position']],
                    #                                    [mpc_cmd_des['velocity']]),
                    #                                    axis=0)
                if not (mpc_qdd_des == self.prev_mpc_qdd_des).all(): #Note: this is not right way to check that command is updated 

                    mpc_cmd_des = self.robot_command_filter.integrate_acc(mpc_qdd_des, 
                                            self.curr_state_filtered_dict,
                                            dt = mpc_cmd_dt - self.tstep)
                                                             

                self.curr_mpc_command.position = mpc_cmd_des['position']
                self.curr_mpc_command.velocity = mpc_cmd_des['velocity']
                self.curr_mpc_command.effort = mpc_qdd_des#np.zeros(7) #What is the third key here??


                self.pub.publish(self.curr_mpc_command)
                # rospy.loginfo('[MPC]: Command published')
                
                if self.tstep == 0:
                    self.start_t = rospy.get_time()

                self.tstep = rospy.get_time() - self.start_t
                self.prev_mpc_qdd_des = mpc_qdd_des
                
                # ee_error,_ = self.controller.rollout_fn.current_cost(curr_state_tensor.unsqueeze(0))
                # ee_error = [x.detach().cpu().item() for x in ee_error]
                # rospy.loginfo(["{:.3f}".format(x) for x in ee_error]) #, "{:.3f}".format(mpc_control.mpc_dt)
                
                if self.debug:
                    self.log_data(qdd_des, [0., 0., 0.])

            elif self.curr_state_raw is None:
                rospy.loginfo('[MPC]: Waiting for state')
            
            elif self.curr_ee_goal is None:
                rospy.loginfo('[MPC]: Waiting for goal`')

            self.rate.sleep()

        self.close()
    

    def initialize_mpc_controller(self):

        with open(self.mpc_yml_file) as file:
            self.exp_params = yaml.load(file, Loader=yaml.FullLoader)

        float_dtype = torch.float32
        if(self.exp_params['float_dtype'] == 'float16'):
            float_dtype = torch.bfloat16
        use_cuda = torch.cuda.is_available() if self.exp_params['use_cuda'] else False
        device = torch.device('cuda', self.exp_params['cuda_device_num']) if use_cuda else torch.device('cpu')

        
        mpc_tensor_dtype = {'device':device, 'dtype':float_dtype}
        rollout_fn = ArmReacherCollisionNN(self.exp_params, device=device, float_dtype=float_dtype, world_params=None)

        mppi_params = self.exp_params['mppi']

        dynamics_model = rollout_fn.dynamics_model
        if(self.exp_params['control_space'] == 'pos'):
            self.exp_params['model']['max_acc'] = 3.0
        #Create controller
        mppi_params['d_action'] = dynamics_model.d_action
        mppi_params['action_lows'] = -self.exp_params['model']['max_acc'] * torch.ones(dynamics_model.d_action, device=device, dtype=float_dtype)
        mppi_params['action_highs'] = self.exp_params['model']['max_acc'] * torch.ones(dynamics_model.d_action, device=device, dtype=float_dtype)
        # init_q = torch.tensor(sim_params['init_state'], device=device)
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
        self.control_process = ControlProcess(self.controller)
        self.controller.rollout_fn.dynamics_model.robot_model.load_lxml_objects()
    
    def log_data(self, qdd_des, ee_error):
        self.robot_q_list_r.append(self.curr_state_raw.position)
        self.robot_qd_list_r.append(self.curr_state_raw.velocity)
        self.robot_qd_des_list_r.append(self.curr_state_raw.effort)
        
        self.robot_q_list_f.append(self.curr_state_filtered.position)
        self.robot_qd_list_f.append(self.curr_state_filtered.velocity)
        self.robot_qd_des_list_f.append(self.curr_state_filtered.effort)
        
        self.command_q_list.append(self.curr_mpc_command.position)
        self.command_qdd_list.append(qdd_des)

        self.ee_dist_err_list.append(ee_error[0])
        self.ee_rot_err_list.append(ee_error[1])

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
                    qdd_cmd = self.command_qdd_list,
                    ee_dist_err = self.ee_dist_err_list,
                    ee_rot_err = self.ee_rot_err_list)
            print('Logs dumped')

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
    rospy.init_node("mpc_controller", anonymous=True)

    # mpc_yml_file = join_path(mpc_configs_path(), robot_params['mpc_yml'])
    mpc_yml_file = os.path.abspath(rospy.get_param('~mpc_yml_file'))
    joint_states_topic = rospy.get_param('~joint_states_topic')
    joint_command_topic = rospy.get_param('~joint_command_topic')
    ee_goal_topic = rospy.get_param('~ee_goal_topic')
    control_freq = rospy.get_param('~control_freq')
    debug = rospy.get_param('~debug')
    joint_names = rospy.get_param('~joint_names')

    mpc_controller = MPCController(joint_states_topic,
                                   joint_command_topic,
                                   ee_goal_topic,
                                   mpc_yml_file,
                                   x_des_list,
                                   control_freq,
                                   debug,
                                   joint_names)

    # while not rospy.is_shutdown():
    #     rospy.spin()
    # rospy.loginfo("""WARNING: This example will move the robot! \n
    #               Please make sure to have the user stop button at hand! \n
    #               Press Enter to continue... \n""")
    # input()
    rospy.loginfo('[MPC]: Initiating Control Loop')
    mpc_controller.control_loop_linear()