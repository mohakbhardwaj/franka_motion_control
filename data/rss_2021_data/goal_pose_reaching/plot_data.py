#!/usr/bin/env python

import numpy as np
# import seaborn as sns
import matplotlib.pyplot as plt
plt.style.use('seaborn-paper')
from differentiable_robot_model.differentiable_robot_model import DifferentiableRobotModel
from differentiable_robot_model.coordinate_transform import matrix_to_quaternion, quaternion_to_matrix
# from stochastic_control.mpc_tools.cost import PoseCost

import torch


# sns.set_theme()
data = np.load('./video_2_data/mpc_data.npz')
urdf_path = "/home/mohak/workspace/stochastic_control/content/assets/urdf/franka_description/franka_panda_no_gripper.urdf"

fig1, ax1 = plt.subplots(5,1)
colors = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#e6ab02', '#a6761d']

for i in range(data['q_robot_r'].shape[1]):
# i = 3
    # if i == 0:
    # ax[0].plot(data['q_robot_r'][:,i], color=colors[i], label='joint_{}_raw'.format(i))
    ax1[0].plot(data['tsteps'], data['q_robot_f'][:,i], color= colors[i])
    ax1[0].plot(data['tsteps'], data['q_cmd'][:,i], color=colors[i], linestyle='dashed', label='joint_{}'.format(i))

    # else:
    #     ax1[0].plot(data['q_robot_r'][:,i])
    #     ax1[0].plot(data['q_robot_f'][:,i])

    # ax1[1].plot(data['tsteps'], data['qd_robot_r'][:,i], color=colors[i])
    ax1[1].plot(data['tsteps'], data['qd_robot_f'][:,i], color=colors[i])
    ax1[1].plot(data['tsteps'], data['qd_cmd'][:,i], color=colors[i], linestyle='dashed')

    ax1[2].plot(data['tsteps'], data['q_robot_f'][:,i], color= colors[i])
    ax1[2].plot(data['tsteps'], data['q_robot_r'][:,i], color= colors[i], linestyle='dashed')

    ax1[3].plot(data['tsteps'], data['qd_robot_f'][:,i], color= colors[i])
    ax1[3].plot(data['tsteps'], data['qd_robot_r'][:,i], color= colors[i], linestyle='dashed')

    ax1[4].plot(data['tsteps'], data['qdd_cmd'][:,i], color=colors[i])

ax1[0].set_title('Robot Joint Positions')
ax1[1].set_title('Robot Joint Velocities')
ax1[2].set_title('Filtered v/s raw (positions)')
ax1[3].set_title('Filtered v/s raw (velocities)')
# ax1[3].set_title('MPC Commanded Joint Positions')
ax1[4].set_title('MPC Commanded Joint Accs')

#Load robot model
fig2 = plt.figure(figsize=(3,2)) #xyz 
ax2 = fig2.add_subplot(1,1,1)
fig3 = plt.figure(figsize=(3,2)) #qx, qy, qz
ax3 = fig3.add_subplot(1,1,1)
fig4 = plt.figure(figsize=(3,2)) #l2 error
ax4 = fig4.add_subplot(1,1,1)
fig5 = plt.figure(figsize=(3,2)) #quaternion error
ax5 = fig5.add_subplot(1,1,1)

tensor_args = {'device':"cpu", 'dtype':torch.float32}
robot_model = DifferentiableRobotModel(urdf_path, None, 
                    tensor_args=tensor_args)


# vec_weight = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
# weight = [1.0,1.0]
# position_gaussian_params = {'n':0, 'c':0.2, 's':0.0, 'r':10.0}
# orientation_gaussian_params =  {'n':0, 'c':0.2, 's':0.0, 'r':10.0}
# hinge_val =  -1
# convergence_val =  [0.0, 0.0] # orientation, position

# tensor_args = {'device':"cpu", 'dtype':torch.float32}
# pose_cost_fn =  PoseCost(weight, vec_weight, position_gaussian_params, 
#                          orientation_gaussian_params, tensor_args, 
#                          hinge_val, convergence_val)


#Run FK to get ee pos, quat
q = torch.tensor(data['q_robot_r'])
qd = torch.tensor(data['qd_robot_r'])
ee_pos, ee_rot = robot_model.compute_forward_kinematics(q, qd, link_name="ee_link")
ee_quat = matrix_to_quaternion(ee_rot)

ee_goal_pos_torch = torch.tensor(data['ee_goal_pos'])
ee_goal_quat_torch = torch.tensor(data['ee_goal_quat'])
ee_goal_rot_torch = quaternion_to_matrix(torch.tensor(data['ee_goal_quat']))




#distance error
ee_dist_error = torch.norm(ee_goal_pos_torch - ee_pos, dim=-1)
#quaternion error
term1 = torch.norm(ee_goal_quat_torch - ee_quat, dim=-1)
term2 = torch.norm(ee_goal_quat_torch + ee_quat, dim=-1)
ee_quat_error = (100.0 / np.sqrt(2.0)) * torch.min(term1, term2)


#vertical lines for goal changes
xcoords =  np.arange(data['tsteps'][0], data['tsteps'][-1]+1, 12)
for xc in xcoords:
    ax2.axvline(x=xc, linestyle='dotted', color="#808080", linewidth=0.6)
    ax3.axvline(x=xc, linestyle='dotted', color="#808080", linewidth=0.6)
    ax4.axvline(x=xc, linestyle='dotted', color="#808080", linewidth=0.6)
    ax5.axvline(x=xc, linestyle='dotted', color="#808080", linewidth=0.6)
# print(data['tsteps'])
# num_lines = int(data['tsteps'][-1] / 12.0)
# print(num_lines)
# cost, ee_rot_error, ee_dist_error = pose_cost_fn.forward(ee_pos, ee_rot, ee_goal_pos_torch, ee_goal_rot_torch)
# preint(ee_rot_error.shape, ee_dist_error.shape)

cart_labels = {'x': '#1b9e77', 'y': '#d95f02', 'z': '#7570b3'}
quat_labels = {'qx': '#1b9e77', 'qy': '#d95f02', 'qz': '#7570b3', 'qw': '#e7298a'}
for i in range(3):
    key = list(cart_labels.keys())[i]
    ax2.plot(data['tsteps'], ee_pos[:,i], label=key, color=cart_labels[key])
    ax2.plot(data['tsteps'], data['ee_goal_pos'][:,i], linestyle='dashed', label=key+"_des", color=cart_labels[key], linewidth=0.7)

# ax2.set_title('EE cartesian')
ax2.legend()
handles, labels = ax2.get_legend_handles_labels()
ax2.legend_.remove()
    
fig2.legend(handles,labels,loc='upper center',ncol=3,bbox_to_anchor=(0.5, 1.0),shadow=False, facecolor='white')


for i in range(4):
    key = list(quat_labels.keys())[i]
    ax3.plot(data['tsteps'], ee_quat[:,i], label=key, color=quat_labels[key])
    ax3.plot(data['tsteps'], data['ee_goal_quat'][:,i], linestyle='dashed', label=key+"_des", color=quat_labels[key], linewidth=0.7)
# ax3.set_title('quaternion')
ax3.legend()
handles, labels = ax3.get_legend_handles_labels()
ax3.legend_.remove()
    
fig3.legend(handles,labels,loc='upper center',ncol=4,bbox_to_anchor=(0.5, 1.),shadow=False, facecolor='white')


ax4.plot(data['tsteps'], ee_dist_error, linewidth=0.7)
ax4.set_title('L2-error')
ax5.plot(data['tsteps'], ee_quat_error, linewidth=0.7)
ax5.set_title('Quaternion error')

fig2.savefig('franka_real_pose_reaching_xyz_response.pdf', dpi=600, bbox_inches='tight')
fig1.savefig('franka_real_pose_reaching_joint_response.pdf', dpi=600, bbox_inches='tight')
fig3.savefig('franka_real_pose_reaching_quat_response.pdf', dpi=600, bbox_inches='tight')
fig4.savefig('franka_real_pose_reaching_ee_error.pdf', dpi=600, bbox_inches='tight')
fig5.savefig('franka_real_pose_reaching_quat_eror.pdf', dpi=600, bbox_inches='tight')


#calculate errors after settling
xcoords =  np.arange(12, data['tsteps'][-1]+12, 12)
xcoords[-1] = data['tsteps'][-1]
print(data['tsteps'][-1])
print(xcoords)
print(ee_dist_error)
ee_dist_errors_settle = np.interp(xcoords, data['tsteps'], ee_dist_error)
ee_quat_errors_settle = np.interp(xcoords, data['tsteps'], ee_quat_error)

print(ee_dist_errors_settle)
print(ee_quat_errors_settle)

avg_error_dist = np.average(ee_dist_errors_settle)
med_error_dist = np.median(ee_dist_errors_settle)
print('Average position error', avg_error_dist)
print('Median position error', med_error_dist)

avg_error_quat = np.average(ee_quat_errors_settle)
med_error_quat = np.median(ee_quat_errors_settle)
print('Average quaternion error', avg_error_quat)
print('Median quaternion error', med_error_quat)




# ax2[1].set_title('EE ee_quaternion')
# ax2[2].set_title('EE L2 error')
# ax2[3].set_title('EE rot error')
# ax[0].legend()
# ax2[0].legend()
# ax2[1].legend()
# plt.tight_layout()
plt.show()
data.close()