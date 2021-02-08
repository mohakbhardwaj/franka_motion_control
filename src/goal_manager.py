#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import std_msgs.msg


goal_ee_pos_list = [[-0.3554, -0.7373,  0.1479]]
goal_ee_quat_list = [[ 0.1571,  0.8050, -0.5714,  0.0267]]

class GoalManager(object):
    def __init__(self, sub_topic, pub_topic, publish_freq=10):
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.publish_freq = publish_freq
    
        self.pub = rospy.Publisher(self.pub_topic, PoseStamped, queue_size=1, latch=False)
        self.sub = rospy.Subscriber(self.sub_topic, JointState, self.goal_callback)
        self.rate = rospy.Rate(self.publish_freq)

        #TBD: Change to None after state machine node has been written
        self.curr_goal = PoseStamped()
        self.curr_goal.header = std_msgs.msg.Header()
        self.curr_goal.header.stamp = rospy.Time.now()
        self.curr_goal.pose.position.x = goal_ee_pos_list[0][0]
        self.curr_goal.pose.position.y = goal_ee_pos_list[0][1]
        self.curr_goal.pose.position.z = goal_ee_pos_list[0][2]
        self.curr_goal.pose.orientation.x = goal_ee_quat_list[0][0]
        self.curr_goal.pose.orientation.y = goal_ee_quat_list[0][1]
        self.curr_goal.pose.orientation.z = goal_ee_quat_list[0][2]
        self.curr_goal.pose.orientation.w = goal_ee_quat_list[0][3]


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
    publish_freq = rospy.get_param('~goal_pub_freq')
 
    goal_manager = GoalManager("joint_pos_controller/joint_states",
                               "ee_goal",
                                publish_freq)
    
    rospy.loginfo('Initiating Goal Publisher')

    goal_manager.publish_goal_loop()