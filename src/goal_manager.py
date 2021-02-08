#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

class GoalManager(object):
    def __init__(self, sub_topic, pub_topic, publish_freq=10):
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.publish_freq = publish_freq
    
        self.pub = rospy.Publisher(self.pub_topic, PoseStamped, queue_size=1, latch=False)
        self.sub = rospy.Subscriber(self.sub_topic, PoseStamped, self.goal_callback)
        self.rate = rospy.Rate(self.publish_freq)

        self.curr_goal = None

    def goal_callback(self, msg):
        self.curr_goal = msg
    
    def publish_goal_loop(self):
        while not rospy.is_shutdown():
        
            if self.curr_goal is not None:
                rospy.loginfo('Curr Goal:', self.curr_goal)
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