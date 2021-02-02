import rospy
from sensor_msgs.msg import JointState
import stochastic_control
import torch
torch.multiprocessing.set_start_method('spawn',force=True)

class MPCController(object):
    def __init__(self, robot_state_topic, command_topic, mpc_yml_file):
        self.robot_state_topic = robot_state_topic
        self.command_topic = command_topic
        self.mpc_yml_file = mpc_yml_file

        self.pub = rospy.Publisher(self.command_topic, JointState, queue_size=10) #TODO: What should be queue size????
        self.sub = rospy.Subscriber(self.robot_state_topic, JointState, self.state_callback)


        self.curr_robot_state = JointState()
        self.start_control = False


    def state_callback(self, msg):
        self.curr_robot_state = msg
        if self.start_control:
            print('Control has started')
    

