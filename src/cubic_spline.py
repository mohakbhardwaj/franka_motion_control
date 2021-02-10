#!/usr/bin/env python
# import numpy as np
import torch
import time

class CubicSplineInterPolation():
    def __init__(self, tensor_args={'device':'cpu','dtype':torch.float32}):
        self.tensor_args = tensor_args
        self.device = self.tensor_args['device']
        self.float_dtype = self.tensor_args['dtype']

        self.M = torch.tensor([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [1.0, 1.0, 1.0, 1.0],
                            [0.0, 1.0, 2.0, 3.0]], 
                            device=self.device, dtype=self.float_dtype)
        # self.M_inv = np.linalg.inv(self.M)
        self.M_inv = torch.inverse(self.M)
        self.A = None
        self.t1 = 0.0
        self.t2 = 0.0

    # def fit(self, t1, t2, y1, y2, dy1t1, dy2t2):
    #     self.t1 = t1
    #     self.t2 = t2
    #     delta_t = t2 - t1
    #     Y = np.array([y1, dy1t1*delta_t, y2, dy2t2*delta_t])
    #     self.A = self.M_inv @ Y
    #     print(self.A, Y)

    def fit(self, state_1, t1, state_2, t2):
        """
            state_1: [2 x n_dims]
            state_2: [2 x n_dims]
        """
        inp_device = state_1.device
        state_1 = state_1.to(self.device, dtype=self.float_dtype)
        state_2 = state_2.to(self.device, dtype=self.float_dtype)

        self.t1 = t1
        self.t2 = t2
        delta_t = t2 - t1
        state_1[1,:] *= delta_t
        state_2[1,:] *= delta_t
        Y = torch.cat((state_1, state_2), dim=0)
        # print(Y.shape, self.M_inv.shape)
        self.A = torch.matmul(self.M_inv, Y)

        # Minv = torch.tensor(self.M_inv)
        # Ytorch = torch.tensor(Y)
        # st = time.time()
        # A = torch.matmul(Minv, Ytorch)
        # print(time.time()-st)
        # print(self.A, A)


    def get_command(self, t):
        """
            t: float from [self.t1, self.t2]
        """
        if (t < self.t1) or  (t > self.t2):
            return None

        u = (t - self.t1) / (self.t2 - self.t1)
        # u_arr = np.array([[1.0, u, u**2, u**3]])
        u_arr = torch.tensor([[1.0, u, u**2, u**3]], device=self.device, dtype=self.float_dtype)
        pred = torch.matmul(u_arr, self.A)
        return pred 
    
    def command_available(self, t):
        if (t < self.t1) or (t > self.t2):
            return False
        elif self.t2 == 0:
            return False
        return True

    


if __name__ == "__main__":
    import numpy as np
    import matplotlib.pyplot as plt
    import time
    spline = CubicSplineInterPolation()

    test_case = 0
    if test_case == 0:    
        #Test 1D spline generation
        # np.random.seed(0)
        torch.manual_seed(0)
        t1 = 0.0; y1 = 1.0; t2 = 0.5; y2 = 1.0; dy1dt1 = 0.0; dy2dt2_rand = torch.randn(10)#np.random.randn(10)

        for dy2dt2 in dy2dt2_rand:
            
            state_1 = torch.tensor([y1, dy1dt1]).reshape(2,1)
            state_2 = torch.tensor([y2, dy2dt2]).reshape(2,1)
            
            spline.fit(state_1, t1, state_2, t2)

            ts = np.arange(t1, t2, 0.001)
            commands = []

            for t in ts:
                commands.append(spline.get_command(t)[0])

            plt.plot(ts, commands, label="dy2dt2={}".format(np.round(dy2dt2,3)))
        plt.legend()
    
    elif test_case == 1:
        #Spline generation for ND states
        ndims = 7
        t1 = 0; t2 = 1
        torch.manual_seed(0)
        s1 = torch.zeros((2,ndims)) #np.random.randn(2,ndims)
        s2 = torch.randn(2,ndims)
        # s1 = np.array([[0, 0], [0, 0]])
        # s2 = np.array([[0, 0], [1, -1]])
        st = time.time()
        spline.fit(s1, t1, s2, t2)
        print("time taken = {}".format(time.time()-st))

        print('State 1 = {}, State 2 = {}'.format(s1, s2))
        ts = np.arange(t1, t2, 0.001)
        commands = []

        for t in ts:
            commands.append(spline.get_command(t).cpu().numpy())
        commands = np.array(commands)
        commands = commands.reshape(commands.shape[0], ndims)

        for i in range(ndims):
            plt.plot(ts, commands[:,i], label='dim={}'.format(i))
        plt.legend()

    elif test_case == 2:
        # Test spline fitting on robot data
        data = np.load('../data/mpc_data.npz')
        fig, ax = plt.subplots(2,1)
        ndims = 7

        for i in range(ndims):
            ax[0].plot(data['q_cmd'][:,i], label='joint={}'.format(i) )

        # for qdd in        
        ax[0].legend()
    plt.show()
