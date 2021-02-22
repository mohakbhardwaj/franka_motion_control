#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from differentiable_robot_model.differentiable_robot_model import DifferentiableRobotModel
from differentiable_robot_model.coordinate_transform import matrix_to_quaternion, quaternion_to_matrix

import torch


# ptcld_data = np.load('pointcloud.npz')
# print(ptcld_data['data'])
# input('....')
data = np.load('mpc_data.npz')
urdf_path = "/home/mohak/workspace/stochastic_control/content/assets/urdf/franka_description/franka_panda_no_gripper.urdf"


fig, ax = plt.subplots(5,1)
fig2, ax2 = plt.subplots(3,1)
colors = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#e6ab02', '#a6761d']

for i in range(data['q_robot_r'].shape[1]):
# i = 3
    # if i == 0:
    # ax[0].plot(data['q_robot_r'][:,i], color=colors[i], label='joint_{}_raw'.format(i))
    ax[0].plot(data['tsteps'], data['q_robot_f'][:,i], color= colors[i], label='joint_{}_filtered'.format(i))
    ax[0].plot(data['tsteps'], data['q_cmd'][:,i], color=colors[i], linestyle='dashed', label='joint_{}_command'.format(i))

    # else:
    #     ax[0].plot(data['q_robot_r'][:,i])
    #     ax[0].plot(data['q_robot_f'][:,i])

    # ax[1].plot(data['tsteps'], data['qd_robot_r'][:,i], color=colors[i])
    ax[1].plot(data['tsteps'], data['qd_robot_f'][:,i], color=colors[i])
    ax[1].plot(data['tsteps'], data['qd_cmd'][:,i], color=colors[i], linestyle='dashed')

    ax[2].plot(data['tsteps'], data['q_robot_f'][:,i], color= colors[i])
    ax[2].plot(data['tsteps'], data['q_robot_r'][:,i], color= colors[i], linestyle='dashed')

    ax[3].plot(data['tsteps'], data['qd_robot_f'][:,i], color= colors[i])
    ax[3].plot(data['tsteps'], data['qd_robot_r'][:,i], color= colors[i], linestyle='dashed')

    ax[4].plot(data['tsteps'], data['qdd_cmd'][:,i], color=colors[i])

ax[0].set_title('Robot Joint Positions')
ax[1].set_title('Robot Joint Velocities')
ax[2].set_title('Filtered v/s raw (positions)')
ax[3].set_title('Filtered v/s raw (velocities)')
# ax[3].set_title('MPC Commanded Joint Positions')
ax[4].set_title('MPC Commanded Joint Accs')

#Load robot model
tensor_args = {'device':"cpu", 'dtype':torch.float32}

robot_model = DifferentiableRobotModel(urdf_path, None, 
                    tensor_args=tensor_args)

#Run FK to get ee pos, quat
q = torch.tensor(data['q_robot_r'])
qd = torch.tensor(data['qd_robot_r'])
ee_pos, ee_rot = robot_model.compute_forward_kinematics(q, qd, link_name="ee_link")
ee_quat = matrix_to_quaternion(ee_rot)
print(ee_pos.shape, ee_quat.shape)

ee_goal_pos_torch = torch.tensor(data['ee_goal_pos'])
ee_error = torch.norm(ee_goal_pos_torch - ee_pos, dim=-1)
print(ee_pos.shape, ee_error.shape)


cart_labels = {'x': '#1b9e77', 'y': '#d95f02', 'z': '#7570b3'}
quat_labels = {'x': '#1b9e77', 'y': '#d95f02', 'z': '#7570b3', 'w': '#e7298a'}
for i in range(3):
    key = list(cart_labels.keys())[i]
    ax2[0].plot(data['tsteps'], ee_pos[:,i], label=key, color=cart_labels[key])
    ax2[0].plot(data['tsteps'], data['ee_goal_pos'][:,i], linestyle='dashed', label=key+"_des", color=cart_labels[key])

for i in range(4):
    key = list(quat_labels.keys())[i]
    ax2[1].plot(data['tsteps'], ee_quat[:,i], label=key, color=quat_labels[key])
    ax2[1].plot(data['tsteps'], data['ee_goal_quat'][:,i], linestyle='dashed', label=key+"_des", color=quat_labels[key])

ax2[2].plot(data['tsteps'], ee_error)

ax2[0].set_title('EE cartesian')
ax2[1].set_title('EE ee_quaternion')
ax2[2].set_title('EE L2 error')

ax[0].legend()
ax2[0].legend()
ax2[1].legend()
plt.tight_layout()
plt.show()
data.close()