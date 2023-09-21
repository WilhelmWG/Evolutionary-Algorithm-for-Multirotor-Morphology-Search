import control as ct
import numpy as np
import MultiRotorDynamics as mrd
import plotting as plt

#Physical Constants
g = 9.81
m_centroid = 0.5
m_rotor = 0.1
m_IMU = 0.1
m_dep_cam = 0.1
d = 0.1
C_q = 0.001 #drag_coefficient
C_t = 0.01 #Thrust coefficient


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
obst_wf = np.ones((3,3))*2
obst_wf[2,2] = 5
obst_wf[1,1] = 4
obst_wf[0,0] = 5


def main():
    rotors = []
    dep_cams = []
    IMU = mrd.IMU(m_IMU,np.array([0,0,np.pi/2],dtype=float),np.array([0,0,0],dtype=float),gyro_bias,magnet_bias, k_a,k_m,k_b)
    #Normal Quadrotor rotors
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([d,0,0],dtype=float),-16,C_q,C_t))
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,d,0],dtype=float),16.5,C_q,C_t))
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([-d,0,0],dtype=float),-16.5,C_q,C_t))
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,-d,0],dtype=float),16.5,C_q,C_t))

    dep_cams.append(mrd.DepthCamera(m_dep_cam, rot_vec=np.array([0,0,0],dtype=float), t_vec=np.array([0,0,d],dtype=float),AoV=AoV,K = K,res = res))
    
    quad = mrd.MultiRotor(m_centroid, rot_vec=np.array([0,0,0],dtype=float),t_vec=np.array([1,1,1],dtype=float), ang_vel=np.array([0,0,0],dtype=float), rotors=rotors, dep_cams = dep_cams, IMU = IMU)
    print(f"Quadrotor Inertial Tensor: \n{quad.calculate_inertial_tensor()}")
    print(f"Sum Of Forces Body Frame: {quad.calculate_sum_of_forces_bf()}")
    print(f"Sum of Torque From Thrust: {quad.calculate_torque_from_thrust_bf()}")
    print(f"Sum of Torque From Thrust: {quad.calculate_torque_from_gravity_bf()}")
    print(f"Sum of Reaction Torque: {quad.calculate_reaction_torque_bf()}")
    for i in range(500):
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
    print(quad.IMU.R_est)
    print(quad.IMU.gyro_bias_est)
    plt.plot_attitude(quad.rot_vec_history, quad.IMU.rot_vec_est_history, delta_t)
    plt.plot_position_3d(quad.t_vec_history,obst_wf)


if __name__ == "__main__":
    main()