import control as ct
import numpy as np
import classes as cs

g = 9.81
m_centroid = 0.5
m_rotor = 0.1
d = 0.1
C_q = 1 #drag_coefficient
C_t = 1 #Thrust coefficient



def main():
    rotors = []

    #Normal Quadrotor rotors
    rotors.append(cs.Rotor(m_rotor,[0,0,0],[d,0,0],C_q,C_t))
    rotors.append(cs.Rotor(m_rotor,[0,0,0],[0,d,0],C_q,C_t))
    rotors.append(cs.Rotor(m_rotor,[0,0,0],[-d,0,0],C_q,C_t))
    rotors.append(cs.Rotor(m_rotor,[0,0,0],[0,-d,0],C_q,C_t))
    
    quad = cs.QuadRotor(m_centroid, rot_vec=[0,0,0],t_vec=[1,1,1], rotors=rotors)

    return

if __name__ == "__main__":
    main()