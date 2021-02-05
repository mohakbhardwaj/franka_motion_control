#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


data = np.load('mpc_data.npz')


fig, ax = plt.subplots(3,1)

for i in range(data['q_robot'].shape[1]):
    ax[0].plot(data['q_robot'][:,i])
    ax[1].plot(data['q_cmd'][:,i])
    ax[2].plot(data['qdd_cmd'][:,i])

ax[0].set_title('Robot Joint Positions')
ax[1].set_title('MPC Commanded Joint Positions')
ax[2].set_title('MPC Commanded Joint Accs')

plt.show()
data.close()