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
    def __init__(self,m,rot_vec,t_vec,gyro_bias,magnet_bias,k_a,k_m,k_b):
        super().__init__(m,rot_vec,t_vec)
        self.gyro_bias = gyro_bias
        self.magnet_bias = magnet_bias
        self.k_a = k_a
        self.k_m = k_m
        self.k_b = k_b
        self.R_est = np.eye(3) #combined bodyframe + limbframe
        self.ang_vel_est = np.zeros((3,))
        self.gyro_bias_est = np.zeros((3,))
        

    
    def get_accelerometer_measurement_lf(self, forces_bf,total_mass): 
        forces_lf = self.R.T@forces_bf
        return forces_lf / total_mass
    
    def get_gyroscope_measurement_lf(self,ang_vel_bf): 
        return self.R.T@(ang_vel_bf) + self.gyro_bias
    
    def get_magnetometer_measurement_lf(self, body_frame_R):
        A_m_wf = np.array([1,0,0],dtype = float)
        A_m_lf = self.R_est.T@A_m_wf
        m_IMU = body_frame_R.T@self.R.T@A_m_wf + self.magnet_bias
        return  m_IMU,A_m_lf

    def update_estimates(self, forces_bf, ang_vel_bf, total_mass, body_frame_R,delta_t):
        acc_meas = self.get_accelerometer_measurement_lf(forces_bf, total_mass)
        ang_vel_meas = self.get_gyroscope_measurement_lf(ang_vel_bf)
        m_IMU, A_m_lf = self.get_magnetometer_measurement_lf(body_frame_R)
        
        z_wf = np.array([0,0,1],dtype = float)
        z_lf_est = self.R_est.T@z_wf
        
        
        alpha = ut.skew((self.k_a/g**2*ut.skew(z_lf_est)@acc_meas + self.k_m/np.linalg.norm(A_m_lf)**2*ut.skew(A_m_lf)@m_IMU))
        # self.gyro_bias_est += np.reshape(self.k_b@alpha*delta_t,(3,))
        self.R_est += (self.R_est@ut.skew(ang_vel_meas-self.gyro_bias_est)-alpha)*delta_t
        self.R_est = R.from_matrix(self.R_est).as_matrix()

        
        







class DepthCamera(Limb):
    def __init__(self,m,rot_vec,t_vec,AoV,K,res):
        super().__init__(m,rot_vec,t_vec)
        self.AoV = AoV # Radians [Horizontal, Vertical, Diagonal]
        self.K = K #intrinsic matrix
        self.res = res
        self.depth_frame = np.zeros((res[0],res[1]))
        
    def project_point(self, point_lf):
        u_hom = self.K@point_lf
        
        if point_lf[2] > 0:
            infront = True
        else: infront = False

        u = np.array([u_hom[0]/u_hom[2],u_hom[1]/u_hom[2]])
        u = np.round(u)
        return u, infront

    def set_depth_frame(self, obst_bf):
        obst_lf = ut.coordinate_transformation(self.T,obst_bf)
        depth_frame = np.zeros((self.res[0],self.res[1]))
        for i in range(obst_lf.shape[1]):
            u,infront = self.project_point(obst_lf[:3,i])
            depth = np.linalg.norm(obst_lf[:3,i]) #Don't know if depth = euclid distance or simply Z
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
    
    def calculate_torque_from_gravity_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.cross(rotor.t_vec,np.reshape(self.R.T@np.array([0,0,rotor.m*g]),(3,))) # Cross displacement with Force
        
        for dep_cam in self.dep_cams:
            sum_torque += np.cross(dep_cam.t_vec,np.reshape(self.R.T@np.array([0,0,dep_cam.m*g]),(3,))) # Cross displacement with Force
        
        sum_torque += np.cross(self.IMU.t_vec,np.reshape(self.R.T@np.array([0,0,self.IMU.m*g]),(3,))) 
        
        return sum_torque
    
    def calculate_reaction_torque_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.reshape(rotor.get_force_bf()*rotor.C_q/rotor.C_t,(3,))*np.sign(rotor.get_rps())
        return sum_torque
    
    def calculate_sum_of_torques_bf(self):
        return self.calculate_reaction_torque_bf()+self.calculate_torque_from_thrust_bf()+self.calculate_torque_from_gravity_bf()

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
        forces_bf = self.calculate_sum_of_forces_bf()
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
        self.t_vec_dot += self.R@forces_bf*delta_t/self.total_mass #1b)
        self.R_dot = self.R@ut.skew(self.ang_vel) #1c)
        
        self.time += delta_t

        #update values
        self.T = ut.transformation_matrix(self.R,self.t_vec)
        self.t_vec_history = np.append(self.t_vec_history,np.reshape(self.t_vec,(3,1)),1)
        self.rot_vec_history = np.append(self.rot_vec_history,np.reshape(self.rot_vec,(3,1)),1)
        self.set_depth_frames(obst_wf)
        self.depth_frames = self.get_depth_frames()
        self.IMU.update_estimates(forces_bf, self.ang_vel, self.total_mass,self.R,delta_t)

