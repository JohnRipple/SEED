import numpy as np
#This file shows how to open and use the variables stored by numpy
data = np.load('camera_distort_matrices.npz')
print(data['mtx'])