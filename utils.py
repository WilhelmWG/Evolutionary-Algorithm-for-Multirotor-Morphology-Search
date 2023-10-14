import numpy as np
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def unskew(R):
    w = np.array([-R[1,2], R[0,2], -R[0,1]])
    return w

#Makes transformation matrix from rotation matrix and translation vector
def transformation_matrix(R,t):
    T = np.zeros((4,4))
    T[:3,:3] = R
    T[:3,3] = t
    T[3,3] = 1
    return T

     
#Performs a coordinate transformation on the points (3, n) using T (4,4)
def coordinate_transformation(T, points):
    if not (points.shape[0] == 4):
        points = np.append(points,np.ones((1,points.shape[1])),0)
    new_frame_points = np.linalg.solve(T,points)
    return new_frame_points
    
#Calculates focal_length given AoV and sensor_sizes along [Horizontal, Vertical, Diagonal]
def calculate_focal_length(AoV, sensor_size):
        for i in range(np.size(AoV)):
            if not (AoV[i] == None):
                return (sensor_size[i]/2)/np.tan(AoV[i]/2)
        return None