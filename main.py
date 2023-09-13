# import control as ct
import numpy as np
import classes as cs

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
    IMU = cs.IMU(m_IMU,np.array([0,0,0],dtype=float),np.array([0,0,0],dtype=float))
    #Normal Quadrotor rotors
    rotors.append(cs.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([d,0,0],dtype=float),2,C_q,C_t))
    rotors.append(cs.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,d,0],dtype=float),30,C_q,C_t))
    rotors.append(cs.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([-d,0,0],dtype=float),2,C_q,C_t))
    rotors.append(cs.Rotor(m_rotor,np.array([0,0,0],dtype=float),np.array([0,-d,0],dtype=float),20,C_q,C_t))
    dep_cams.append(cs.DepthCamera(m_dep_cam, rot_vec=np.array([0,0,0],dtype=float), t_vec=np.array([0,0,0],dtype=float),AoV=40))
    
    quad = cs.QuadRotor(m_centroid, rot_vec=[0,0,0],t_vec=[1,1,1], rotors=rotors, dep_cams = dep_cams, IMU = IMU)
    print(quad.calculate_inertial_tensor())
    print(quad.calculate_sum_of_forces())
    print(quad.calculate_torque_from_thrust())

if __name__ == "__main__":
    main()