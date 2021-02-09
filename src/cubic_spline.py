#!/usr/bin/env python
import numpy as np


class CubicSplineInterPolation():
    def __init__(self, ndims):
        self.ndims = ndims
        self.M = np.array([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [1.0, 1.0, 1.0, 1.0],
                            [0.0, 1.0, 2.0, 3.0]], dtype=np.float32)
        self.M_inv = np.linalg.inv(self.M)
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
        y1 = state_1[0, 0:self.ndims]
        y2 = state_2[0, 0:self.ndims]
        dy1dt1 = state_1[1, 0:self.ndims]
        dy2dt2 = state_2[1, 0:self.ndims]

        self.t1 = t1
        self.t2 = t2
        delta_t = t2 - t1
        Y = np.hstack([y1, dy1dt1*delta_t, y2, dy2dt2*delta_t])
        self.A = self.M_inv @ Y
        print(self.A, Y)


    def get_command(self, t):
        if t > self.t2:
            return None

        u = (t - self.t1) / (self.t2 - self.t1)
        u_arr = np.array([1.0, u, u**2, u**3])
        
        return self.A.dot(u_arr)
    


if __name__ == "__main__":

    import matplotlib.pyplot as plt

    test_case = 1
    if test_case == 0:    
        spline = CubicSplineInterPolation(ndims=1)

        np.random.seed(0)
        t1 = 0.0; y1 = 1.0; t2 = 0.5; y2 = 1.0; dy1dt1 = 0.0; dy2dt2_rand = np.random.randn(10)

        for dy2dt2 in dy2dt2_rand:
            
            state_1 = np.array([y1, dy1dt1])
            state_2 = np.array([y2, dy2dt2])
            
            spline.fit(state_1, t1, state_2, t2)

            ts = np.arange(t1, t2, 0.001)
            commands = []

            for t in ts:
                commands.append(spline.get_command(t))

            plt.plot(ts, commands, label="dy2dt2={}".format(np.round(dy2dt2,3)))
        plt.legend()
    elif test_case == 1:
        spline = CubicSplineInterPolation(ndims=7)

        data = np.load('../data/mpc_data.npz')
        fig, ax = plt.subplots(2,1)
        # for i in range(data['q_cmd'].shape[1]):
        i = 3
        ax[0].plot(data['q_cmd'][:,i])
        curr_t = 0
        dt = 0.001

        for qdd in        



plt.show()
