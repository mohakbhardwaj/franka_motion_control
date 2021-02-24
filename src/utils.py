import numpy as np

def clip_arr(arr, clip_val):
    clip_arr = np.where(np.abs(arr) > clip_val, arr, 0.0)
    return clip_arr