# import control as ct
import numpy as np
import QuadrotorDynamics as qd

g = 9.81
m_centroid = 0.5
m_rotor = 0.1
m_IMU = 0.1
m_dep_cam = 0.1
d = 0.1
C_q = 0.001 #drag_coefficient
C_t = 0.01 #Thrust coefficient



def main():
    rotors = []
    dep_cams = []
    IMU = qd.IMU(m_IMU,np.array([0,0,0],dtype=float),np.array([0,0,0],dtype=float))
    #Normal Quadrotor rotors
    rotors.append(qd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([d,0,0],dtype=float),-10,C_q,C_t))
    rotors.append(qd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,d,0],dtype=float),10,C_q,C_t))
    rotors.append(qd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([-d,0,0],dtype=float),-10,C_q,C_t))
    rotors.append(qd.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,-d,0],dtype=float),10,C_q,C_t))
    dep_cams.append(qd.DepthCamera(m_dep_cam, rot_vec=np.array([0,0,0],dtype=float), t_vec=np.array([0,0,0],dtype=float),AoV=40))
    
    quad = qd.QuadRotor(m_centroid, rot_vec=np.array([0,0,0],dtype=float),t_vec=np.array([1,1,1],dtype=float), ang_vel=np.array([0,0,0],dtype=float), rotors=rotors, dep_cams = dep_cams, IMU = IMU)
    print(f"Quadrotor Inertial Tensor: \n{quad.calculate_inertial_tensor()}")
    print(f"Sum Of Forces: {quad.calculate_sum_of_forces_bf()}")
    print(f"Torque From Thrust: {quad.calculate_torque_from_thrust_bf()}")
    print(f"Reaction Torque: {quad.calculate_reaction_torque_bf()}")
    for i in range(100):
        quad.simulate_timestep(0.01)
    breakpoint()

if __name__ == "__main__":
    main()