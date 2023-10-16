import control as ct
import numpy as np
import scipy as sp
import utils as ut
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R
from scipy.integrate import solve_ivp
from scipy.misc import derivative


g = 9.81

class Limb:
    def __init__(self, m, rot_vec, t_vec):
        self.m = m
        self.rot_vec = rot_vec 
        self.t_vec = t_vec
        self.R = R.from_euler("zxy",rot_vec).as_matrix() #Rotation Matrix
        self.T = ut.transformation_matrix(self.R,self.t_vec)

class Rotor(Limb):
    def __init__(self, m, rot_vec, t_vec, rps, C_q, C_t, sigma):
        super().__init__(m,rot_vec,t_vec)
        self.rps = rps #Rotations per second
        self.sigma = sigma #which way the rotor rotates
        self.C_q = C_q
        self.C_t = C_t
    
    #returns the force in body frame
    def get_force_bf(self):
        force_rf = self.C_t*self.rps**2*np.array([0,0,1])*np.sign(self.rps)*self.sigma
        force_rf = np.reshape(force_rf,(3,1))
        force_bf = self.R@force_rf
        # print(f"force_bf : {force_bf}")
        return force_bf
    
    def get_rps(self):
        return self.rps
    
    def set_rps(self, rps):
        self.rps = rps

    def force_to_rps(self, force):
        # print(np.sqrt(np.abs(force)/self.C_t)*np.sign(force)*self.sigma)
        return np.sqrt(np.abs(force)/self.C_t)*np.sign(force)*self.sigma

    def set_force(self, force):
        self.set_rps(self.force_to_rps(force))





class IMU(Limb):
    def __init__(self,m,rot_vec,t_vec,gyro_bias,magnet_bias,k_a,k_m,k_b):
        super().__init__(m,rot_vec,t_vec)
        self.gyro_bias = gyro_bias
        self.magnet_bias = magnet_bias
        self.k_a = k_a
        self.k_m = k_m
        self.k_b = k_b
        self.R_est = np.eye(3) #combined bodyframe + limbframe
        self.rot_vec_est = np.zeros((3,))
        self.ang_vel_est = np.zeros((3,))
        self.gyro_bias_est = np.zeros((3,))
        self.rot_vec_est_history = np.reshape(rot_vec,(3,1))
        

    
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
        RR = R.from_matrix(self.R_est)
        self.R_est = RR.as_matrix()
        R_bf = R.from_matrix(self.R@self.R_est@self.R.T)
        self.rot_vec_est = R_bf.as_euler("zxy")
        self.rot_vec_est_history = np.append(self.rot_vec_est_history,np.reshape(self.rot_vec_est,(3,1)),1)






class TrajectoryPlanner():
    def __init__(self, delta_t, max_time, x_d_lambda,b1_d_lambda):
        self.delta_t = delta_t
        self.max_time = max_time
        self.t = np.linspace(0,max_time,int(max_time/delta_t)+1) 
        self.x_d = x_d_lambda(self.t)
        self.x_dot_d = derivative(x_d_lambda,self.t,dx=1e-6)
        self.x_dot_dot_d = derivative(x_d_lambda,self.t,dx=1e-6,n=2)
        self.b1_d = b1_d_lambda(self.t)

        self.prev_R_d = np.eye(3)
        self.prev_ang_vel_d = np.zeros((3,))
        self.ang_vel_dot_d = np.zeros((3,))

        
    def get_trajectory(self,time):
        return self.x_d[:,int(time/self.delta_t)],self.x_dot_d[:,int(time/self.delta_t)],self.x_dot_dot_d[:,int(time/self.delta_t)], self.b1_d[:,int(time/self.delta_t)]






class Controller():
    def __init__(self,k_x,k_v,k_R,k_omega, TrajectoryPlanner: TrajectoryPlanner,rotors: List[Rotor]):
        self.k_x = k_x
        self.k_v = k_v
        self.k_R = k_R
        self.k_omega = k_omega
        self.allocation_matrix = self.calculate_allocation_matrix(rotors)
        self.TP = TrajectoryPlanner
        
    
    def update_trajectory(self,time):
        x_d, x_dot_d, x_dot_dot_d, b1_d = self.TP.get_trajectory(time)
        self.traj = [x_d, x_dot_d, x_dot_dot_d, b1_d]
    

    def calculate_errors(self, m, time, R_mat, ang_vel, t_vec, t_vec_dot):
        R_x = R.from_euler("zxy",np.array([0,np.pi,0])).as_matrix()
        R_y = R.from_euler("zxy",np.array([0,0,np.pi/2])).as_matrix()
        R_z = R.from_euler("zxy",np.array([np.pi/2,0,0])).as_matrix()
        print(R_mat)
        R_mat = (R_z@R_x@R_mat)# This is because paper uses R as rotation from body frame to inertial frame as opposed to my implementation
        # R_mat = R_mat.T

        delta_t = self.TP.delta_t
        e3 = np.array([0,0,1],dtype = float)
        
        self.update_trajectory(time)
        x_d, x_dot_d, x_dot_dot_d, b1_d = self.traj

        #Positional errors
        e_x = t_vec - x_d
        e_v = t_vec_dot - x_dot_d

        #Calculate R_d
        ctrl = -self.k_x*e_x - self.k_v*e_v-m*g*e3+m*x_dot_dot_d
        b3_d = -ctrl/np.linalg.norm(ctrl)
        cross31 = np.cross(b3_d,b1_d)
        b2_d = cross31/np.linalg.norm(cross31)
        b1_d_new = np.cross(b2_d,b3_d)
        R_d = np.array([b1_d_new, b2_d,b3_d])
        
        
        #Calculate ang_vel
        delta_R_d = R_d-self.TP.prev_R_d
        R_d_dot = delta_R_d/delta_t
        ang_vel_d = ut.unskew(R_d.T@R_d_dot)
        delta_ang_vel_d = ang_vel_d-self.TP.prev_ang_vel_d
        self.TP.ang_vel_dot_d = delta_ang_vel_d/self.TP.delta_t

        #angular errors
        delta_R = 1/2*(R_d.T@R_mat-R_mat.T@R_d)
        e_R = ut.unskew(delta_R)
        e_omega = ang_vel-R_mat.T@R_d@ang_vel_d
        
        #update previous with current
        self.TP.prev_R_d = R_d
        self.TP.prev_ang_vel_d = ang_vel_d
         # e_omega too big, makes no sense
        print(R_d)
        print(R_mat)
        print(f"errors: {e_x, e_v, e_R, e_omega}\n")
        return e_x, e_v, e_R, e_omega
        
      
    def calculate_forces(self,m,e_x,e_v,e_R, e_omega, R_mat, ang_vel, J):
        R_x = R.from_euler("zxy",np.array([0,np.pi,0])).as_matrix()
        R_y = R.from_euler("zxy",np.array([0,0,np.pi])).as_matrix()
        R_z = R.from_euler("zxy",np.array([np.pi/2,0,0])).as_matrix()
        # R_mat = (R_x@R_mat@R_x.T).T # This is because paper uses R as rotation from body frame to inertial frame as opposed to my implementation
        # R_mat = R_mat.T
        R_mat = (R_z@R_x@R_mat)
        R_d = self.TP.prev_R_d
        e3 = np.array([0,0,1],dtype = float)
        x_dot_dot_d = self.traj[2]
        
        f = -(-self.k_x*e_x-self.k_v*e_v-m*g*e3+m*x_dot_dot_d)@R_mat@e3 
        M = -self.k_R*e_R-self.k_omega*e_omega+ut.skew(ang_vel)@J@ang_vel-J@(ut.skew(ang_vel)@R_mat.T@R_d@self.TP.prev_ang_vel_d-R_mat.T@R_d@self.TP.ang_vel_dot_d)
        return f, M
    
    def calculate_allocation_matrix(self, rotors):
        e3 = np.array([0,0,1],dtype = float)
        n_r = 0
        allocation_matrix = np.array([])
        for rotor in rotors:
            n_r += 1
            gamma = rotor.R@e3
            gamma_proj = e3.T@gamma #Forces projected on b3 in body frame
            theta = (ut.skew(rotor.t_vec)+rotor.C_q/rotor.C_t*rotor.sigma*np.eye(3))@gamma
            allocation_matrix = np.append(allocation_matrix,np.insert(theta,0,gamma_proj))
        allocation_matrix = np.reshape(allocation_matrix,(4,n_r)).T
        print(allocation_matrix) 
        return allocation_matrix

    def force_allocation(self, f, M):
        
        forces = np.insert(M,0,f)
        rotor_forces = np.linalg.solve(self.allocation_matrix,forces) #allocation_matrix@[f1,f2,f3,f4] = [f,M] solves for f1,f2,f3,f4
        print(f"[f,M] : {forces}")
        return rotor_forces

    def get_rotor_forces(self,m,J,time,R_mat,ang_vel,t_vec,t_vec_dot):
        e_x, e_v, e_R, e_omega = self.calculate_errors(m,time,R_mat,ang_vel,t_vec,t_vec_dot)
        f, M = self.calculate_forces(m,e_x,e_v, e_R, e_omega, R_mat, ang_vel, J)
        
        rotor_forces = self.force_allocation(f, M)
        
        
        # print(f"rotor_forces : {rotor_forces}")
        return rotor_forces


    




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
    def __init__(self, m,  rot_vec, t_vec, ang_vel, rotors: List[Rotor], dep_cams: List[DepthCamera], IMU: IMU, Controller: Controller):
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
        self.Controller = Controller
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
        print(f"Sum of Forces: {sum_force}")
        sum_force -= self.R.T@np.array([0,0,self.total_mass*g])#np.reshape(self.R@rotor.get_force_bf(),(3,)) #This doesn't make any sense
        print(f"Sum of Forces: {sum_force}")
        return sum_force

    def calculate_torque_from_thrust_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.cross(rotor.t_vec,np.reshape(rotor.get_force_bf(),(3,))) # Cross Displacement with Force
        return sum_torque
    
    def calculate_torque_from_gravity_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.cross(rotor.t_vec,np.reshape(self.R.T@np.array([0,0,-rotor.m*g]),(3,))) # Cross displacement with Force
        
        for dep_cam in self.dep_cams:
            sum_torque += np.cross(dep_cam.t_vec,np.reshape(self.R.T@np.array([0,0,-dep_cam.m*g]),(3,))) # Cross displacement with Force
        
        sum_torque += np.cross(self.IMU.t_vec,np.reshape(self.R.T@np.array([0,0,-self.IMU.m*g]),(3,))) 
        
        return sum_torque
    
    def calculate_reaction_torque_bf(self):
        sum_torque = np.zeros((3,))
        for rotor in self.rotors:
            sum_torque += np.reshape(rotor.get_force_bf()*rotor.C_q/rotor.C_t,(3,))*np.sign(rotor.get_rps())
        return sum_torque
    
    def calculate_sum_of_torques_bf(self):
        print(f"M_real: {self.calculate_reaction_torque_bf()+self.calculate_torque_from_thrust_bf()}")
        return self.calculate_reaction_torque_bf()+self.calculate_torque_from_thrust_bf()#+self.calculate_torque_from_gravity_bf()

    def set_rotor_forces(self,rotor_forces):
        i = 0
        for rotor in self.rotors:
            rotor.set_force(rotor_forces[i])
            i += 1

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
        print(self.R@forces_bf)
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
        rotor_forces = self.Controller.get_rotor_forces(self.total_mass, self.J, self.time,self.R,self.ang_vel,self.t_vec,self.t_vec_dot)
        self.set_rotor_forces(rotor_forces)
        self.T = ut.transformation_matrix(self.R,self.t_vec)
        self.t_vec_history = np.append(self.t_vec_history,np.reshape(self.t_vec,(3,1)),1)
        self.rot_vec_history = np.append(self.rot_vec_history,np.reshape(self.rot_vec,(3,1)),1)
        self.set_depth_frames(obst_wf)
        self.depth_frames = self.get_depth_frames()
        self.IMU.update_estimates(forces_bf, self.ang_vel, self.total_mass,self.R,delta_t)
        

        



