import numpy as np
import ros_numpy 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String

def joint_state_to_dict(msg):
    return {'position': np.array(msg.position), 
            'velocity': np.array(msg.velocity)} 
            #'acceleration': np.array(msg.effort) 

def dict_to_joint_state(dict):
    msg = JointState()
    msg.position = dict['position']
    msg.velocity = dict['velocity']
    # msg.effort = dict['acceleration'] 
    return msg

def pose_stamped_to_np(msg):
    pos = np.array([msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z])
    quat = np.array([msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w])
    return pos, quat

def pointcloud2_to_np(msg):
    pc = ros_numpy.numpify(msg)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    return np.array(points)