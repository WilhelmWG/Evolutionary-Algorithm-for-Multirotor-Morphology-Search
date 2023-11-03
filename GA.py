import numpy as np
import pygad as ga
import MultiRotorDynamics as MRD
import plotting as plt
from MotorRotorAnalysis import motor_dict
g = 9.81
#simulation parameters
obst_wf = np.ones((3,3))*2
obst_wf[2,2] = 5
obst_wf[1,1] = 4
obst_wf[0,0] = 5
delta_t = 0.01 #seconds
max_time = 20

#Trajectory
x_d = lambda t : np.array([0.4*t,0.4*np.sin(np.pi*t),0.6*np.cos(np.pi*t)])# x_d = lambda t : np.array([0*t,1*t,1*t])#
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])# b1_d = lambda t : np.array([1*t,0*t,0*t])# b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])

m_IMU = 0.05
m_dep_cam = 0.1
m_centroid = 0.122
m_rotor = 0.0 #accounted for
m_total = m_centroid + m_rotor*4 + m_IMU + m_dep_cam #migrate from here
d = 0.315
#IMU parameters
k_a = 0.01
k_m = 0.01
k_b = np.reshape(np.array([0.1,0.1,0.1],dtype=float),(1,3))*0.01 #k_a/10
gyro_bias = np.array([0,0,0],dtype=float)
magnet_bias = np.array([0,0,0],dtype=float)

#Camera Properties
AoV = np.array([39.6,27.0,46.8]) * np.pi/180 #[Horizontal, Vertical, Diagonal]
sensor_size = np.array([36,24,43.3]) #[Horizonta, Vertical, Diagonal]
res = np.array([1920,1080])
K = np.array([[1200, 0, res[0]/2],
              [0,1200, res[1]/2],
              [0,0,1]]) #Camera Intrinsics

#controller parameters
k_x = 16*m_total
k_v = 5.6*m_total
k_R = 8.81*0.1
k_omega = 2.54*0.1

#If the user did not assign the initial population to the initial_population parameter,
# the initial population is created randomly based on the gene_space parameter.
# Moreover, the mutation is applied based on this parameter.

#[num_rotors, num_depcams]
gene_space = [range(4,9), [1,2]]
#[num_rotors, num_depcams, n_rmax*[num_comb,yaw,pitch,roll,x,y,z]]
for i in range(8):
    gene_space.append(range(20))
    for i in range(3):
        gene_space.append({'low': -np.pi/2, 'high': np.pi/2})
    for i in range(3):
        gene_space.append({'low': -d, 'high': d})
    gene_space.append([-1,1])

#[num_rotors, num_depcams, n_rmax*[num_comb,yaw,pitch,roll,x,y,z],n_depmax[yaw,pitch,roll,x,y,z]]
for i in range(2):
    for i in range(3):
        gene_space.append({'low': -np.pi/2, 'high': np.pi/2})
    for i in range(3):
        gene_space.append({'low': -d, 'high': d})

num_generations = 50
num_parents_mating = 5
sol_per_pop = 20
num_genes = len(gene_space)

def fitness_func(ga_instance, solution, solution_idx):
    quad = load_quad_from_sol(solution)
    quad.simulate(max_time,delta_t,obst_wf)

    fitness = np.linalg.norm(quad.t_vec_history - quad.Controller.TP.x_d)
    return fitness


ga_instance = ga.GA(num_generations=num_generations,
                    num_parents_mating=num_parents_mating,
                    sol_per_pop=sol_per_pop,
                    num_genes=num_genes,
                    gene_space=gene_space)

ga_instance.run()

def load_quad_from_sol(solution):
    num_rotors = solution[0]
    num_depcams = solution[1]
    rotors = []
    dep_cams = []
    for i in range(num_rotors):
        comb_num = solution[2+7*i]
        m = motor_dict[comb_num]["mass"]
        rot_vec = solution[3+7*i:6+7*i]
        t_vec = solution[6+7*i:9+7*i]
        sigma = solution[9+7*i]

        rotor = MRD.Rotor(m, rot_vec, t_vec, rps=0, sigma = sigma, motor_prop_comb_num = comb_num)
        rotors.append(rotor)
    
    for i in range(num_depcams):
        rot_vec = solution[2+7*num_rotors+6*i:2+7*num_rotors+6*i+3]
        t_vec = solution[2+7*num_rotors+6*i+3:2+7*num_rotors+6*i+6]
        
    IMU = MRD.IMU(m_IMU,np.array([0,0,np.pi/2],dtype=float),np.array([0,0,0],dtype=float),gyro_bias,magnet_bias, k_a,k_m,k_b)
    