import control as ct
import numpy as np
import scipy as sp
import utils as ut
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R
from scipy.integrate import solve_ivp


g = 9.81


class Limb:
    def __init__(self, m, rot_vec, t_vec):
        self.m = m
        self.rot_vec = rot_vec 
        self.t_vec = t_vec
        self.R = R.from_euler("zxy",rot_vec).as_matrix() #Rotation Matrix
        self.T = ut.transformation_matrix(self.R,self.t_vec)

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
    
    def get_rps(self):
        return self.rps
    
    def set_rps(self, rps):
        self.rps = rps

class IMU(Limb):
    def __init__(self,m,rot_vec,t_vec):
        super().__init__(m,rot_vec,t_vec)

    def get_measurements(self, forces_bf): pass #Not really necessary as forces are determined exactly in MultiRotor.calculate_sum_of_forces_bf(), but might be nice to keep things conceptually valid


class DepthCamera(Limb):
    def __init__(self,m,rot_vec,t_vec,AoV,K,res):
        super().__init__(m,rot_vec,t_vec)
        self.AoV = AoV # Radians [Horizontal, Vertical, Diagonal]
        self.K = K #intrinsic matrix
        self.res = res
        self.depth_frame = np.zeros((res[0],res[1]))
        
    def project_point(self, point_cf):
        u_hom = self.K@point_cf
        
        if point_cf[2] > 0:
            infront = True
        else: infront = False

        u = np.array([u_hom[0]/u_hom[2],u_hom[1]/u_hom[2]])
        u = np.round(u)
        return u, infront

    def set_depth_frame(self, obst_bf):
        obst_cf = ut.coordinate_transformation(self.T,obst_bf)
        depth_frame = np.zeros((self.res[0],self.res[1]))
        for i in range(obst_cf.shape[1]):
            u,infront = self.project_point(obst_cf[:3,i])
            depth = np.linalg.norm(obst_cf[:3,i]) #Don't know if depth = euclid distance or simply Z
            if (u[0] < self.res[0] and u[0] > 0) and (u[1] < self.res[1] and u[1] > 0) and infront:
                depth_frame[int(u[0]),int(u[1])] = depth
        self.depth_frame = depth_frame    

    def get_depth_frame(self):
        return self.depth_frame
    


class MultiRotor:
    def __init__(self, m, rot_vec, t_vec, ang_vel, rotors: List[Rotor], dep_cams: List[DepthCamera], IMU: IMU):
        self.time = 0
        self.m = m
        
        self.rot_vec = rot_vec #Euler angles
        self.rot_vec_dot = np.zeros((3,))
        self.t_vec = t_vec #translation vector
        self.t_vec_dot = np.zeros((3,))
        self.ang_vel = ang_vel #Angular velocity
        self.ang_vel_dot = np.zeros((3,))
        self.R = R.from_euler("zxy",rot_vec).as_matrix() #Rotation Matrix
        self.R_dot = np.zeros((3,3))
        self.T = ut.transformation_matrix(self.R,t_vec)

        self.rotors = rotors
        self.dep_cams = dep_cams
        self.IMU = IMU
        self.J = self.calculate_inertial_tensor()

        self.t_vec_history = np.reshape(t_vec,(3,1))
        self.rot_vec_history = np.reshape(rot_vec,(3,1))

        self.total_mass = self.calculate_total_mass()
        
    def calculate_total_mass(self):
        m = 0
        m += self.m
        m += self.IMU.m
        for rotor in self.rotors:
            m += rotor.m
        for dep_cam in self.dep_cams:
            m += dep_cam.m
        return m
    
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
    
    def calculate_sum_of_forces_bf(self):
        sum_force = np.zeros((3,))
        for rotor in self.rotors:
            sum_force += np.reshape(rotor.get_force_bf(),(3,))
        sum_force -= self.R.T@np.array([0,0,self.total_mass*g])#np.reshape(self.R@rotor.get_force_bf(),(3,)) #This doesn't make any sense
        return sum_force

    def calculate_torque_from_thrust_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.cross(rotor.t_vec,np.reshape(rotor.get_force_bf(),(3,))) # Cross Displacement with Force
        return sum_torque
    
    def calculate_reaction_torque_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.reshape(rotor.get_force_bf()*rotor.C_q/rotor.C_t,(3,))*np.sign(rotor.get_rps())
        return sum_torque
    
    def calculate_sum_of_torques_bf(self):
        return self.calculate_reaction_torque_bf()+self.calculate_torque_from_thrust_bf()

    def set_depth_frames(self, obst_wf):
        T = ut.transformation_matrix(self.R,self.t_vec)
        obst_bf = ut.coordinate_transformation(T,obst_wf)
        for dep_cam in self.dep_cams:
            dep_cam.set_depth_frame(obst_bf)
    
    def get_depth_frames(self):
        depth_frames = []
        for dep_cam in self.dep_cams:
            depth_frames.append(dep_cam.depth_frame)
        return depth_frames

    def simulate_timestep(self,delta_t,obst_wf):
        #Pose
        self.t_vec += self.t_vec_dot*delta_t #1a)
        self.R += self.R_dot*delta_t

        #Double transform to maintain SO(3) (Kinda cheating), Should not be included when using ode45
        self.rot_vec = R.from_matrix(self.R).as_euler("zxy")
        self.R  = R.from_euler("zxy",self.rot_vec).as_matrix()
        
        #Angular velocity
        self.ang_vel += self.ang_vel_dot*delta_t
        
        #velocity
        self.ang_vel_dot = np.linalg.solve(self.J,(-np.cross(self.ang_vel,self.J@self.ang_vel)+self.calculate_sum_of_torques_bf())) #1d)
        self.t_vec_dot += self.R@self.calculate_sum_of_forces_bf()*delta_t/self.total_mass #1b)
        self.R_dot = self.R@ut.skew(self.ang_vel) #1c)
        
        self.time += delta_t

        #update values
        self.T = ut.transformation_matrix(self.R,self.t_vec)
        self.t_vec_history = np.append(self.t_vec_history,np.reshape(self.t_vec,(3,1)),1)
        self.rot_vec_history = np.append(self.rot_vec_history,np.reshape(self.rot_vec,(3,1)),1)
        self.set_depth_frames(obst_wf)
        self.depth_frames = self.get_depth_frames()
        # self.IMU.update_estimates(self.calculate_sum_of_forces_bf(),self.calculate_sum_of_torques_bf())

