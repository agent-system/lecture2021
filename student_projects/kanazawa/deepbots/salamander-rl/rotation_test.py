from scipy.spatial.transform import Rotation
import numpy as np

def show_vec(mat):
    rot = Rotation.from_dcm(mat)
    print(rot.as_rotvec()[1])

mat1 = np.array([[-1.0,  0.0,  0.0],
                 [ 0.0,  1.0,  0.0],
                 [ 0.0,  0.0, -1.0]])
show_vec(mat1)


mat2 = np.array([[ 1.0,  0.0,  0.0],
                 [ 0.0,  1.0,  0.0],
                 [ 0.0,  0.0,  1.0]])
show_vec(mat2)


mat3 = np.array([[ 0.0,  0.0,  1.0],
                 [ 0.0,  1.0,  0.0],
                 [-1.0,  0.0,  0.0]])
show_vec(mat3)


mat4 = np.array([[ 0.0,  0.0,  -1.0],
                 [ 0.0,  1.0,  0.0],
                 [1.0,  0.0,  0.0]])
show_vec(mat4)




