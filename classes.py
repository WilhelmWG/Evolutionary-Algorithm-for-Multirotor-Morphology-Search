# import control as ct
import numpy as np
import scipy as sp
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R




class Limb:
    def __init__(self, m, rot_vec, t_vec):
        self.m = m
        self.rot_vec = rot_vec 
        self.t_vec = t_vec

class Rotor(Limb):
    def __init__(self, m, rot_vec, t_vec, rps, C_q, C_t):
        super().__init__(m,rot_vec,t_vec)
        self.rps = rps #Rotations per second
        self.C_q = C_q
        self.C_t = C_t
    #returns the force in body frame
    def get_force_bf(self):
        force_rf = self.C_t*self.rps**2*np.array([0,0,1])
        force_rf = np.reshape(force_rf,(3,1))
        rot_mat = R.from_euler("zxy",self.rot_vec)
        rot_mat = rot_mat.as_matrix()
        force_bf = rot_mat@force_rf
        return force_bf
    
    def get_torque_bf(self):
        return

class IMU(Limb):
    def __init__(self,m,rot_vec,t_vec):
        super().__init__(m,rot_vec,t_vec)

class DepthCamera(Limb):
    def __init__(self,m,rot_vec,t_vec,AoV):
        super().__init__(m,rot_vec,t_vec)
        self.AoV = AoV

class QuadRotor:
    def __init__(self, m, rot_vec, t_vec, rotors: List[Rotor], dep_cams: List[DepthCamera], IMU: IMU):
        self.m = m
        self.rot_vec = rot_vec #
        self.t_vec = t_vec
        self.rotors = rotors
        self.dep_cams = dep_cams
        self.IMU = IMU
    def calculate_inertial_tensor(self):
        J = np.zeros((3,3),dtype=float)
        for rotor in self.rotors:
            t_vec = np.reshape(rotor.t_vec,(3,1))
            J += rotor.m*(sp.linalg.norm(t_vec)**2*np.eye(3)-(t_vec)@(t_vec.T))
            
        for dep_cam in self.dep_cams:
            t_vec = np.reshape(dep_cam.t_vec,(3,1))
            J += dep_cam.m*(sp.linalg.norm(t_vec)**2*np.eye(3)-(t_vec)@(t_vec.T))
        
        t_vec = np.reshape(self.IMU.t_vec,(3,1))
        J += self.IMU.m*(sp.linalg.norm(t_vec)**2*np.eye(3)-(t_vec)@(t_vec.T))
        
        return J
    
    def calculate_sum_of_forces(self):
        sum_force = np.zeros((3,))
        for rotor in self.rotors:
            sum_force += np.reshape(rotor.get_force_bf(),(3,))
        return sum_force

    def calculate_torque_from_thrust(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.cross(rotor.t_vec,np.reshape(rotor.get_force_bf(),(3,)))
        return sum_torque