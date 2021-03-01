#!/usr/bin/env python
import rospy
import os
# import tf2_ros
import numpy as np
import torch
from sensor_msgs.msg import PointCloud2

from stochastic_control_conversions import *

class EnvManager(object):
    def __init__(self, pointcloud_topic, filepath):
        self.filename = filepath
        self.pointcloud_topic = pointcloud_topic

        # if self.pointcloud_topic is not None:
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        # self.curr_pointcloud = rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=None)
        # self.save_pointcloud()
        self.pointcloud_received = False

    # def save_pointcloud(self, msg):
    def pointcloud_callback(self, msg):
        #only static scenes for now
        if not self.pointcloud_received:
            self.curr_pointcloud = msg
            pc_data = torch.tensor(pointcloud2_to_np(self.curr_pointcloud))
            pc_labels = torch.ones(pc_data.shape[0],1)
            robot_camera_pose = torch.tensor([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) #assuming pointcloud is base_link frame
            camera_data = {'pc': pc_data, 'pc_seg': pc_labels, 'robot_camera_pose': robot_camera_pose}
            torch.save(camera_data, self.filename)

            # self.controller.rollout_fn.voxel_collision_cost.coll.set_scene(data, np.zeros_like(data))
            self.pointcloud_received = True
            rospy.loginfo("Pointcloud saved")


if __name__ == "__main__":
    rospy.init_node("env_manager", anonymous=True)
    topic = rospy.get_param('~pointcloud_topic')
    filename = rospy.get_param('~outfile_name')
    print(filename)
    filename = "/home/mohak/catkin_ws/src/franka_motion_control/data/rss_2021/obstacle_avoidance/environments/" + filename
    env_manager = EnvManager(topic, filename)
    rospy.spin()