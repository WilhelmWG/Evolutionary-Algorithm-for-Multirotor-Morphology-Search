import numpy as np
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def transformation_matrix(R,t):
    T = np.zeros((4,4))
    T[:3,:3] = R
    T[:3,3] = t
    T[3,3] = 1
    return T

def calculate_focal_length(AoV, sensor_size):
        for i in range(np.size(AoV)):
            if not (AoV[i] == None):
                return (sensor_size[i]/2)/np.tan(AoV[i]/2)
        return None