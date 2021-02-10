#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


data = np.load('mpc_data.npz')


fig, ax = plt.subplots(6,1)
fig2, ax2 = plt.subplots(2,1)

for i in range(data['q_robot_r'].shape[1]):
# i = 3
    if i == 0:
        ax[0].plot(data['q_robot_r'][:,i], label='raw')
        ax[0].plot(data['q_robot_f'][:,i], label='filtered')
    else:
        ax[0].plot(data['q_robot_r'][:,i])
        ax[0].plot(data['q_robot_f'][:,i])

    ax[1].plot(data['qd_robot_r'][:,i])
    ax[1].plot(data['qd_robot_f'][:,i])

    ax[2].plot(data['qd_des_robot_r'][:,i])
    # ax[2].plot(data['qd_des_robot_f'][:,i])

    ax[3].plot(data['q_cmd'][:,i])
    ax[4].plot(data['qdd_cmd'][:,i])

ax[0].set_title('Robot Joint Positions')
ax[1].set_title('Robot Joint Velocities (measured)')
ax[2].set_title('Robot Joint Velocities (desired')
ax[3].set_title('MPC Commanded Joint Positions')
ax[4].set_title('MPC Commanded Joint Accs')

ax2[0].plot(data['ee_dist_err'])
ax2[1].plot(data['ee_rot_err'])
ax2[0].set_title('EE dist error')
ax2[1].set_title('EE rot error')

ax[0].legend()
plt.tight_layout()
plt.show()
data.close()