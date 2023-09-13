# import control as ct
import numpy as np
import scipy as sp
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R
from scipy.integrate import solve_ivp


g = 9.81
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

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
    def __init__(self, m, rot_vec, t_vec, ang_vel, rotors: List[Rotor], dep_cams: List[DepthCamera], IMU: IMU):
        self.time = 0
        self.m = m
        
        self.rot_vec = rot_vec #
        self.rot_vec_dot = np.zeros((3,))
        self.t_vec = t_vec
        self.t_vec_dot = np.zeros((3,))
        self.ang_vel = ang_vel
        self.ang_vel_dot = np.zeros((3,))
        
        self.rotors = rotors
        self.dep_cams = dep_cams
        self.IMU = IMU
        self.J = self.calculate_inertial_tensor()
        
    
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
        
        self.J = J
    
    def calculate_sum_of_forces_bf(self):
        sum_force = np.zeros((3,))
        for rotor in self.rotors:
            sum_force += np.reshape(rotor.get_force_bf(),(3,))
        rot_mat = R.from_euler("zxy",self.rot_vec)
        rot_mat = rot_mat.as_matrix()
        sum_force -= np.reshape(rot_mat@rotor.get_force_bf(),(3,))
        return sum_force

    def calculate_torque_from_thrust_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.cross(rotor.t_vec,np.reshape(rotor.get_force_bf(),(3,)))
        return sum_torque
    
    def calculate_reaction_torque_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.reshape(rotor.get_force_bf()*rotor.C_q/rotor.C_t,(3,))*np.sign(rotor.rps)
        return sum_torque
    
    def calculate_sum_of_torques_bf(self):
        return self.calculate_reaction_torque_bf()+self.calculate_torque_from_thrust_bf()

    def simulate_timestep(self,delta_t):
        rot_mat = R.from_euler("zxy",self.rot_vec)
        rot_mat = rot_mat.as_matrix()
        
        
        
        #position
        self.t_vec += self.t_vec_dot*delta_t #1a)
        self.rot_vec += self.rot_vec_dot*delta_t
        
        #Angular velocity
        self.ang_vel += self.ang_vel_dot*delta_t
        
        #velocity
        self.ang_vel_dot = -np.linalg.inv(self.J)@(np.cross(self.ang_vel,self.J@self.ang_vel)+self.calculate_sum_of_torques_bf()) #SUPA SLOW, MAKE IT LINEAR SOLVE INSTEAD
        self.t_vec_dot += rot_mat@self.calculate_sum_of_forces_bf()*delta_t #1b)
        R_dot = rot_mat@skew(self.ang_vel)
        rot_dot = R.from_matrix(R_dot)
        self.rot_vec_dot = rot_dot.as_euler("zxy")*delta_t
        
        self.time += delta_t
