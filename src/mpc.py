import rospy
from sensor_msgs.msg import JointState
# import stochastic_control
from stochastic_control.mpc_tools.rollout.arm_reacher import ArmReacher
from stochastic_control.mpc_tools.control import MPPI, StompMPPI
from stochastic_control.mpc_tools.utils.state_filter import JointStateFilter
from stochastic_control.mpc_tools.utils.mpc_process_wrapper import ControlProcess

import time
import torch
torch.multiprocessing.set_start_method('spawn',force=True)

class MPCController(object):
    def __init__(self, robot_state_topic, command_topic, mpc_yml_file, goal_state_list):
        rospy.init_node("mpc_controller", anonymous=True)

        self.robot_state_topic = robot_state_topic
        self.command_topic = command_topic
        self.mpc_yml_file = mpc_yml_file
        # with open(mpc_yml_file) as file:
        #     self.self.exp_params = yaml.load(file, Loader=yaml.FullLoader)

        self.goal_state_list = goal_state_list
        self.curr_goal_idx = 0
        self.curr_goal = self.goal_state_list[self.curr_goal_idx]

        #Initialize MPC controller
        self.initialize_mpc_controller()
        self.control_process.update_goal(goal_state=self.curr_goal)
        self.controller.rollout_fn.update_goal(goal_state=self.curr_goal)

        #Initialize ROS
        self.pub = rospy.Publisher(self.command_topic, JointState, queue_size=1, latch=False)
        self.sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)

        #Variables for book-keeping
        self.curr_robot_state = JointState()
        self.curr_mpc_command = JointState()
        self.start_control = False
        self.zero_acc = np.zeros(7)
        self.tstep = 0
        self.start_t = time.time()

        rospy.spin()


    def state_callback(self, msg):
        self.curr_robot_state = msg
        curr_state_np = np.hstack((msg.position, msg.velocity, self.zero_acc))
        curr_state = torch.as_tensor(curr_state_np).unsqueeze(0)
        
        
        if self.start_control:
            next_command, val, info = self.control_process.get_command(self.t_step, curr_state)

            if(self.exp_params['control_space'] == 'acc'):
                qdd_des = np.ravel(next_command[0])
                cmd_des = self.robot_filter.integrate_acc(qdd_des, current_state)#['position']
                # q_des = cmd_des['position']
                self.curr_mpc_command.position = cmd_des['position']
                self.curr_mpc_command.velocity = cmd_des['velocity']
                self.curr_mpc_command.effort = cmd_des['acc']
            
            elif(self.exp_params['control_space'] == 'pos'):
                self.curr_mpc_command.position = np.ravel(next_command[0])
                self.curr_mpc_command.velocity = np.zeros(7)
                self.curr_mpc_command.effor = cmd_des['acc']
            
            self.pub.publish(self.curr_mpc_command)
        else:
            #initialize joint filter
            self.robot_filter = JointStateFilter(curr_state_np)

        self.tstep = time.time() - self.start_t #TODO: Check this versus ros::Time now and state timestamp
    


    def initialize_mpc_controller(self):

        with open(self.mpc_yml_file) as file:
            self.exp_params = yaml.load(file, Loader=yaml.FullLoader)

        float_dtype = torch.float32
        if(self.exp_params['float_dtype'] == 'float16'):
            float_dtype = torch.bfloat16
        use_cuda = torch.cuda.is_available() if self.exp_params['use_cuda'] else False
        device = torch.device('cuda', self.exp_params['cuda_device_num']) if use_cuda else torch.device('cpu')

        
        mpc_tensor_dtype = {'device':device, 'dtype':float_dtype}
        rollout_fn = ArmReacher(self.exp_params, device=device, float_dtype=float_dtype,world_params=world_params)

        mppi_params = self.exp_params['mppi']

        dynamics_model = rollout_fn.dynamics_model
        if(self.exp_params['control_space'] == 'pos'):
            self.exp_params['model']['max_acc'] = 3.0
        #Create controller
        mppi_params['d_action'] = dynamics_model.d_action
        mppi_params['action_lows'] = -self.exp_params['model']['max_acc'] * torch.ones(dynamics_model.d_action, device=device, dtype=float_dtype)
        mppi_params['action_highs'] = self.exp_params['model']['max_acc'] * torch.ones(dynamics_model.d_action, device=device, dtype=float_dtype)
        init_q = torch.tensor(sim_params['init_state'], device=device)
        init_action = torch.zeros((mppi_params['horizon'], dynamics_model.d_action), device=device)
        init_action[:,:] += init_q
        if(self.exp_params['control_space'] == 'acc'):
            mppi_params['init_mean'] = init_action * 0.0 # device=device)
        elif(self.exp_params['control_space'] == 'pos'):
            mppi_params['init_mean'] = init_action 
        mppi_params['rollout_fn'] = rollout_fn
        mppi_params['device'] = device
        mppi_params['float_dtype'] = float_dtype

        self.controller = MPPI(**mppi_params)
        self.control_process = ControlProcess(controller)

        return controller


if __name__ == '__main__':
    goal_list = np.zeros(6)
    mpc_controller = MPCController("joint_pos_controller/joint_states",
                                   "joint_pos_controller/joint_pos_goal",
                                   goal_list)
