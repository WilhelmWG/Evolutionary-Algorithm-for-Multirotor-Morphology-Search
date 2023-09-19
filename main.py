import control as ct
import numpy as np
import MultiRotorDynamics as mrd

g = 9.81
m_centroid = 0.5
m_rotor = 0.1
m_IMU = 0.1
m_dep_cam = 0.1
d = 0.1
C_q = 0.001 #drag_coefficient
C_t = 0.01 #Thrust coefficient



AoV = np.array([39.6,27.0,46.8]) * np.pi/180 #[Horizontal, Vertical, Diagonal]
sensor_size = np.array([36,24,43.3]) #[Horizonta, Vertical, Diagonal]
res = np.array([1920,1080])
K = np.array([[1200, 0, res[0]],
              [0,1200, res[1]],
              [0,0,1]]) #Camera Intrinsics


def main():
    rotors = []
    dep_cams = []
    IMU = mrd.IMU(m_IMU,np.array([0,0,0],dtype=float),np.array([0,0,0],dtype=float))
    #Normal Quadrotor rotors
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([d,0,0],dtype=float),-10,C_q,C_t))
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,d,0],dtype=float),10,C_q,C_t))
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([-d,0,0],dtype=float),-10,C_q,C_t))
    rotors.append(mrd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,-d,0],dtype=float),10,C_q,C_t))

    dep_cams.append(mrd.DepthCamera(m_dep_cam, rot_vec=np.array([0,0,0],dtype=float), t_vec=np.array([0,0,0],dtype=float),AoV=AoV,K = K,res = res))
    
    quad = mrd.MultiRotor(m_centroid, rot_vec=np.array([0,0,0],dtype=float),t_vec=np.array([1,1,1],dtype=float), ang_vel=np.array([0,0,0],dtype=float), rotors=rotors, dep_cams = dep_cams, IMU = IMU)
    print(f"Quadrotor Inertial Tensor: \n{quad.calculate_inertial_tensor()}")
    print(f"Sum Of Forces: {quad.calculate_sum_of_forces_bf()}")
    print(f"Sum of Torque From Thrust: {quad.calculate_torque_from_thrust_bf()}")
    print(f"Sum of Reaction Torque: {quad.calculate_reaction_torque_bf()}")
    for i in range(100):
        quad.simulate_timestep(0.01)
    print(quad.t_vec_history)
    print(quad.rot_vec_history)
    print(quad.T)

if __name__ == "__main__":
    main()