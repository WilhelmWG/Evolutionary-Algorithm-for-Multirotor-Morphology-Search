import control as ct
import numpy as np
import pygad as ga
import MultiRotorDynamics as MRD
import plotting as plt


#Physical Constants
g = 9.81
m_centroid = 0.122
m_rotor = 0.0 #accounted for
m_IMU = 0.05
m_dep_cam = 0.1
m_total = m_centroid + m_rotor*4 + m_IMU + m_dep_cam
d = 0.315
C_q = 8.004e-4 #drag_coefficient
C_t = 1e-2 #Thrust coefficient


#Camera Properties
AoV = np.array([39.6,27.0,46.8]) * np.pi/180 #[Horizontal, Vertical, Diagonal]
sensor_size = np.array([36,24,43.3]) #[Horizonta, Vertical, Diagonal]
res = np.array([1920,1080])
K = np.array([[1200, 0, res[0]/2],
              [0,1200, res[1]/2],
              [0,0,1]]) #Camera Intrinsics

#IMU parameters
k_a = 0.01
k_m = 0.01
k_b = np.reshape(np.array([0.1,0.1,0.1],dtype=float),(1,3))*0.01 #k_a/10
gyro_bias = np.array([0,0,0],dtype=float)
magnet_bias = np.array([0,0,0],dtype=float)


#Simulation parameters
delta_t = 0.01 #seconds
max_time = 20.2
obst_wf = np.ones((3,3))*2
obst_wf[2,2] = 5
obst_wf[1,1] = 4
obst_wf[0,0] = 5

#controller parameters
k_x = 16*m_total
k_v = 5.6*m_total
k_R = 8.81*0.1
k_omega = 2.54*0.1

#Trajectory
x_d = lambda t : np.array([0.4*t+1,0.4*np.sin(np.pi*t)+1,0.6*np.cos(np.pi*t)+1])# x_d = lambda t : np.array([0*t,1*t,1*t])#
# 
# b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])# # b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])


def main():
    rotors = []
    dep_cams = []
    IMU = MRD.IMU(m_IMU,np.array([0,0,np.pi/2],dtype=float),np.array([0,0,0],dtype=float),gyro_bias,magnet_bias, k_a,k_m,k_b)

    TrajectoryPlanner = MRD.TrajectoryPlanner(delta_t,max_time,x_d,b1_d,b3_d)
    Battery = MRD.Battery(0.6,5,6,"big")
    #Normal Quadrotor rotors
    rotors.append(MRD.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([d,0,0],dtype=float),20,-1,0))
    rotors.append(MRD.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,d,0],dtype=float),-20,1,0))
    rotors.append(MRD.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([-d,0,0],dtype=float),20,-1,0))
    rotors.append(MRD.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,-d,0],dtype=float),-20,1,0))
    rotors.append(MRD.Rotor(m_rotor,np.array([0,np.pi/8,0],dtype=float),np.array([+d/np.sqrt(2),-d/np.sqrt(2),-0.25],dtype=float),20,1,16))
    rotors.append(MRD.Rotor(m_rotor,np.array([0,0,np.pi/8],dtype=float),np.array([-d/np.sqrt(2),d/np.sqrt(2),-0.25],dtype=float),20,-1,16))
    
    #Custom
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,0,-np.pi/32],dtype=float),np.array([d,0,0],dtype=float),20,-1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,np.pi/32,0],dtype=float),np.array([0,d,0],dtype=float),-20,1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,0,np.pi/32],dtype=float),np.array([-d,0,0],dtype=float),20,-1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,-np.pi/32,0],dtype=float),np.array([0,-d,0],dtype=float),-20,1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,0,np.pi/3],dtype=float),np.array([d/np.sqrt(2),d/np.sqrt(2),0],dtype=float),20,-1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([-d/np.sqrt(2),-d/np.sqrt(2),0],dtype=float),-20,1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,0,np.pi/2],dtype=float),np.array([-d/np.sqrt(2),d/np.sqrt(2),-0.25],dtype=float),20,-1,16))
    # rotors.append(MRD.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([d/np.sqrt(2),-d/np.sqrt(2),0.25],dtype=float),-20,1,16))
    Controller = MRD.Controller(k_x,k_v,k_R,k_omega,TrajectoryPlanner, rotors)
    dep_cams.append(MRD.DepthCamera(m_dep_cam, rot_vec=np.array([0,0,0],dtype=float), t_vec=np.array([0,0,d],dtype=float),AoV=AoV,K = K,res = res))
    
    quad = MRD.MultiRotor(m_centroid,
                          rot_vec=np.array([0,0,0],dtype=float),
                          t_vec=np.array([1,1,1],dtype=float),
                          ang_vel=np.array([0,0,0],dtype=float),
                          rotors=rotors,
                          dep_cams = dep_cams,
                          IMU = IMU, 
                          Controller = Controller,
                          Battery = Battery)
    
    print(f"Quadrotor Inertial Tensor: \n{quad.calculate_inertial_tensor()}")
    print(f"Sum Of Forces Body Frame: {quad.calculate_sum_of_forces_bf()}")
    print(f"Sum of Torque From Thrust: {quad.calculate_torque_from_thrust_bf()}")
    print(f"Sum of Torque From Thrust: {quad.calculate_torque_from_gravity_bf()}")
    print(f"Sum of Reaction Torque: {quad.calculate_reaction_torque_bf()}")
    # for i in range(int(6)):
    #     quad.simulate_timestep(delta_t,obst_wf)
    if(quad.maxTzi > 2*g*quad.total_mass):
        print(f"{quad.maxTzi} > {2*g*quad.total_mass}")
        for i in range(int(max_time/delta_t)):
            quad.simulate_timestep(delta_t,obst_wf)
        
    
    # print(quad.t_vec_history)
    # print(quad.rot_vec_history)
    print(quad.T)
    # print(np.linalg.norm(quad.T[:,0]))
    # print(np.linalg.norm(quad.T[:,1]))
    # print(np.linalg.norm(quad.T[:,2]))
    # print(np.linalg.norm(quad.T[0,:3]))
    # print(np.linalg.norm(quad.T[1,:3]))
    # print(np.linalg.norm(quad.T[2,:3]))
    # print(quad.dep_cams[0].depth_frame.shape)
    # print(np.max(quad.dep_cams[0].depth_frame))
    # print(np.unravel_index(np.argmax(quad.dep_cams[0].depth_frame),quad.dep_cams[0].depth_frame.shape))
    # print(quad.IMU.R_est)
    # print(quad.IMU.gyro_bias_est)
    plt.plot_attitude(quad.rot_vec_history, quad.IMU.rot_vec_est_history, delta_t)
    plt.plot_position_2d(quad.t_vec_history,delta_t)
    plt.plot_position_3d(quad.t_vec_history,obst_wf)
    plt.plot_MR_design(quad)


if __name__ == "__main__":
    main()