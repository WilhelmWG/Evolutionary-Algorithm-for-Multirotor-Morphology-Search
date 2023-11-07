import numpy as np
import pygad as ga
import MultiRotorDynamics as MRD
import plotting as plt
from MotorRotorAnalysis import motor_dict, battery_dict


g = 9.81
#simulation parameters
obst_wf = np.ones((3,3))*2
obst_wf[2,2] = 5
obst_wf[1,1] = 4
obst_wf[0,0] = 5
delta_t = 0.01 #seconds
max_time = 20

#Trajectory
x_d = lambda t : np.array([0.4*t+1,0.4*np.sin(np.pi*t)+1,0.6*np.cos(np.pi*t)+1])# x_d = lambda t : np.array([0*t,1*t,1*t])#
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])# b1_d = lambda t : np.array([1*t,0*t,0*t])# b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])

m_IMU = 0.02
m_dep_cam = 0.03
m_centroid = 0.122
m_rotor = 0.0 #accounted for
m_total = m_centroid + m_rotor*4 + m_IMU + m_dep_cam #migrate from here
d = 0.3
max_angle = np.pi/2

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
k_x = 16
k_v = 5.6
k_R = 8.81*0.1
k_omega = 2.54*0.1



#GA PARAMS
#If the user did not assign the initial population to the initial_population parameter,
# the initial population is created randomly based on the gene_space parameter.
# Moreover, the mutation is applied based on this parameter.

num_generations = 50
num_parents_mating = 10
sol_per_pop = 50

#[num_rotors, num_depcams]
gene_space = [range(4,9), [1,2]]
n_rotor_max = 8
n_depcam_max = 2
#[num_rotors, num_depcams, n_rmax*[num_comb,yaw,pitch,roll,r,x,y,z,sigma]]
for i in range(n_rotor_max):
    gene_space.append(range(20)) #ncomb
    for i in range(3):
        gene_space.append({'low': -max_angle, 'high': max_angle}) #angles
    gene_space.append({'low': 0, 'high': d}) #absolute value of displacement
    for i in range(3):
        gene_space.append({'low': -1, 'high': 1}) #direction of displacement
    gene_space.append([-1,1]) #sigma

#[num_rotors, num_depcams, n_rmax*[num_comb,yaw,pitch,roll,r,x,y,z,sigma],n_depmax[yaw,pitch,roll,r,x,y,z]]
for i in range(2):
    for i in range(3):
        gene_space.append({'low': -max_angle, 'high': max_angle})#angles
    gene_space.append({'low': 0, 'high': d}) #radius
    for i in range(3):
        gene_space.append({'low': -1, 'high': 1}) #direction of displacement

gene_space.append(range(19))


num_genes = len(gene_space)




def load_MR_from_sol(solution):
    # print(f"solution = {solution}")
    num_rotors = solution[0]
    num_depcams = solution[1]
    rotors = []
    dep_cams = []
    for i in range(int(num_rotors)):
        comb_num = int(solution[2+9*i])
        # print(f"comb_num = {comb_num}")
        m = motor_dict[comb_num]["mass"]
        # print(f"mass = {m}")
        
        rot_vec = solution[3+9*i:6+9*i]
        # print(f"rotvec = {rot_vec}")
        r = solution[6+9*i]
        # print(f"r = {r}")
        t_vec = solution[7+9*i:10+9*i]
        # print(f"t_vec= {t_vec}")
        t_vec = r*(t_vec/np.linalg.norm(t_vec)) #rescale
        # print(f"t_vecscaled = {t_vec}")
        
        sigma = solution[10+9*i]

        rotor = MRD.Rotor(m, rot_vec, t_vec, rps=0, sigma = sigma, motor_prop_comb_num = comb_num)
        rotors.append(rotor)
    
    for i in range(int(num_depcams)):
        rot_vec = solution[2+9*n_rotor_max+7*i:2+9*n_rotor_max+7*i+3]
        r = solution[2+9*n_rotor_max+6*i+3]
        t_vec = solution[2+9*n_rotor_max+7*i+4:2+9*n_rotor_max+7*i+7]
        t_vec = r*(t_vec/np.linalg.norm(t_vec)) #rescale

        dep_cam = MRD.DepthCamera(m_dep_cam,rot_vec,t_vec,AoV,K,res)
        dep_cams.append(dep_cam)

    battery_vals = battery_dict[int(solution[2+9*n_rotor_max+7*n_depcam_max])]
    bat_m = battery_vals["mass"]
    bat_Ah = battery_vals["Ah"]
    bat_S = battery_vals["S"]
    bat_name = battery_vals["name"]

    Battery = MRD.Battery(bat_m,bat_Ah,bat_S,bat_name)
    IMU = MRD.IMU(m_IMU,np.array([0,0,np.pi/2],dtype=float),np.array([0,0,0],dtype=float),gyro_bias,magnet_bias, k_a,k_m,k_b)
    TP = MRD.TrajectoryPlanner(delta_t,max_time,x_d,b1_d)
    Controller = MRD.Controller(k_x,k_v,k_R, k_omega, TP,rotors)
    MR = MRD.MultiRotor(m_centroid,
                          rot_vec=np.array([0,0,0],dtype=float),
                          t_vec=np.array([1,1,1],dtype=float),
                          ang_vel=np.array([0,0,0],dtype=float),
                          rotors=rotors,
                          dep_cams = dep_cams,
                          IMU = IMU, 
                          Controller = Controller,
                          Battery = Battery)
    
    return MR





def fitness_func(ga_instance, solution, solution_idx):
    MR = load_MR_from_sol(solution)
    valid = MR.simulate(max_time,delta_t,obst_wf)
    w_e = 1
    w_A = 100
    if valid:
        fitness = -w_e*np.linalg.norm(MR.t_vec_history - MR.Controller.TP.x_d) + w_A*MR.Battery.currentAh/MR.Battery.maxAh
        # print(f"FITNESS:::::: {fitness}")
    
        if np.isnan(fitness):
            fitness = -1
        print(f"FITNESS:::::: {fitness}")
    else: 
        fitness = -1
    return fitness






def run_ga():

    #Make your own mutation operator
    ga_instance = ga.GA(num_generations=num_generations,
                    num_parents_mating=num_parents_mating,
                    sol_per_pop=sol_per_pop,
                    num_genes=num_genes,
                    gene_space=gene_space,
                    parent_selection_type='tournament',
                    fitness_func=fitness_func,
                    save_best_solutions=True,
                    crossover_type=None,
                    parallel_processing=["process",8])
    ga_instance.summary()
    ga_instance.run()
    return ga_instance


if __name__ == "__main__":
    run_ga()


