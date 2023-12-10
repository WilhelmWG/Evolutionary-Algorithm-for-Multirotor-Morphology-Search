import numpy as np
import pygad as ga
import MultiRotorDynamics as MRD
import plotting as plt
import random

from MotorRotorAnalysis import motor_dict, battery_dict
from Trajs import x_ds, b1_ds, b3_ds
from pygad import utils as ut


g = 9.81
#simulation parameters
delta_t = 0.01 #seconds
max_time = 5

#obstacles (unused)
obst_wf = np.ones((3,3))*2
obst_wf[2,2] = 5
obst_wf[1,1] = 4
obst_wf[0,0] = 5



m_IMU = 0.02
m_dep_cam = 0.03
m_centroid = 0.148
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
k_x_max = 20
k_v_max = 10
k_R_max = 10
k_omega_max = 5
random_yaw = True

#GA PARAMS
#If the user did not assign the initial population to the initial_population parameter,
# the initial population is created randomly based on the gene_space parameter.
# Moreover, the mutation is applied based on this parameter.
num_motor_comb = 20
num_battery_types = 14
num_generations = 50
num_parents_mating = 50
sol_per_pop = 200

#GENE SPACE
#[num_rotors, num_depcams]
gene_space = [range(4,9), [1,2]]
n_rotor_max = 8
n_depcam_max = 2
#[num_rotors, num_depcams, n_rmax*[num_comb,yaw,pitch,roll,r,x,y,z,sigma]]
for i in range(n_rotor_max):
    gene_space.append(range(num_motor_comb)) #ncomb
    for i in range(3):
        gene_space.append({'low': -max_angle, 'high': max_angle}) #angles
    gene_space.append({'low': d/2, 'high': d}) #absolute value of displacement
    for i in range(3):
        gene_space.append({'low': -1, 'high': 1}) #direction of displacement
    gene_space.append([-1,1]) #sigma

#[num_rotors, num_depcams, n_rmax*[num_comb,yaw,pitch,roll,r,x,y,z,sigma],n_depmax[yaw,pitch,roll,r,x,y,z]]
for i in range(n_depcam_max):
    for i in range(3):
        gene_space.append({'low': -max_angle, 'high': max_angle})#angles
    gene_space.append({'low': d/2, 'high': d}) #radius
    for i in range(3):
        gene_space.append({'low': -1, 'high': 1}) #direction of displacement

gene_space.append(range(num_battery_types))

gene_space.append({'low': 0, 'high': k_x_max})
gene_space.append({'low': 0, 'high': k_v_max})
gene_space.append({'low': 0, 'high': k_R_max})
gene_space.append({'low': 0, 'high': k_omega_max})

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

    k_x = solution[2+9*n_rotor_max+7*n_depcam_max+1]
    k_v = solution[2+9*n_rotor_max+7*n_depcam_max+2]
    k_R = solution[2+9*n_rotor_max+7*n_depcam_max+3]
    k_omega = solution[2+9*n_rotor_max+7*n_depcam_max+4]

    # print(f"k_x {k_x}, k_v {k_v}, k_R {k_R}, k_omega {k_omega}")

    Battery = MRD.Battery(bat_m,bat_Ah,bat_S,bat_name)
    IMU = MRD.IMU(m_IMU,np.array([0,0,np.pi/2],dtype=float),np.array([0,0,0],dtype=float),gyro_bias,magnet_bias, k_a,k_m,k_b)
    TP = MRD.TrajectoryPlanner(delta_t,max_time,x_ds,b1_ds,b3_ds,random_yaw)
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
    fitness = 0
    failed_traj_count = 0
    random_init = True
    MR = load_MR_from_sol(solution)
    for i in range(len(MR.Controller.TP.x_ds)-1):
        traj_fitness = 0
        MR.next_trajectory(random_init)
        valid = MR.simulate(max_time,delta_t,obst_wf)
        
        w_t = 1
        w_r = 1 
        w_A = 100

        if valid:
            traj_fitness = -w_t*np.linalg.norm(MR.t_vec_history - MR.Controller.TP.x_d) - w_r*np.linalg.norm(MR.rot_err_history)
            
            # print(f"FITNESS:::::: {fitness}")
        
            if np.isnan(traj_fitness):
                traj_fitness = -100
                failed_traj_count += 1
        else: 
            traj_fitness = -100
            failed_traj_count += 1

        fitness += traj_fitness

    if not np.isnan(w_A*MR.Battery.currentAh/MR.Battery.maxAh) and not (MR.Battery.currentAh/MR.Battery.maxAh == 1):
        fitness += w_A*MR.Battery.currentAh/MR.Battery.maxAh
    

    print(f"Battery left: {MR.Battery.currentAh/MR.Battery.maxAh}")
    print(f"FITNESS:::::: {fitness}")
    print(f"failed trajectories : {failed_traj_count}")
    
    return fitness


def available_motor_combs(S):
    motors = []
    for i in range(num_motor_comb):
        motor_S = motor_dict[i]["S"]
        if S in motor_S:
            motors.append(i)
    return motors

def motor_comb_idxs():
    idxs = []
    for i in range(n_rotor_max):
        idxs.append(2+9*i)
    return idxs


#Slightly modified from PyGAD source
def mutation_by_space_x(offspring,ga_instance):

    """
    Applies the random mutation using the mutation values' space.
    It accepts a single parameter:
        -offspring: The offspring to mutate.
    It returns an array of the mutated offspring using the mutation space.
    """

    # For each offspring, a value from the gene space is selected randomly and assigned to the selected mutated gene.
    for offspring_idx in range(offspring.shape[0]):
        mutation_indices = np.array(random.sample(range(0, ga_instance.num_genes), ga_instance.mutation_num_genes))
        if ga_instance.generations_completed == 0:
            mutation_indices = np.append(mutation_indices, 2+9*n_rotor_max+7*n_depcam_max)
        # print(mutation_indices)
        for gene_idx in mutation_indices:

            if type(ga_instance.random_mutation_min_val) in ga_instance.supported_int_float_types:
                range_min = ga_instance.random_mutation_min_val
                range_max = ga_instance.random_mutation_max_val
            else:
                range_min = ga_instance.random_mutation_min_val[gene_idx]
                range_max = ga_instance.random_mutation_max_val[gene_idx]

            if ga_instance.gene_space_nested:
                # Returning the current gene space from the 'gene_space' attribute.
                if type(ga_instance.gene_space[gene_idx]) in [np.ndarray, list]:
                    curr_gene_space = ga_instance.gene_space[gene_idx].copy()
                else:
                    curr_gene_space = ga_instance.gene_space[gene_idx]

                # If the gene space has only a single value, use it as the new gene value.
                if type(curr_gene_space) in ga.GA.supported_int_float_types:
                    value_from_space = curr_gene_space
                # If the gene space is None, apply mutation by adding a random value between the range defined by the 2 parameters 'random_mutation_min_val' and 'random_mutation_max_val'.
                elif curr_gene_space is None:
                    rand_val = np.random.uniform(low=range_min,
                                                    high=range_max,
                                                    size=1)[0]
                    if ga_instance.mutation_by_replacement:
                        value_from_space = rand_val
                    else:
                        value_from_space = offspring[offspring_idx, gene_idx] + rand_val
                elif type(curr_gene_space) is dict:
                    # The gene's space of type dict specifies the lower and upper limits of a gene.
                    if 'step' in curr_gene_space.keys():
                        # The np.random.choice() and np.random.uniform() functions return a np array as the output even if the array has a single value.
                        # We have to return the output at index 0 to force a numeric value to be returned not an object of type np.ndarray. 
                        # If np.ndarray is returned, then it will cause an issue later while using the set() function.
                        value_from_space = np.random.choice(np.arange(start=curr_gene_space['low'],
                                                                            stop=curr_gene_space['high'],
                                                                            step=curr_gene_space['step']),
                                                                size=1)[0]
                    else:
                        value_from_space = np.random.uniform(low=curr_gene_space['low'],
                                                                high=curr_gene_space['high'],
                                                                size=1)[0]
                else:
                    # Selecting a value randomly based on the current gene's space in the 'gene_space' attribute.
                    # If the gene space has only 1 value, then select it. The old and new values of the gene are identical.
                    if len(curr_gene_space) == 1:
                        value_from_space = curr_gene_space[0]
                    # If the gene space has more than 1 value, then select a new one that is different from the current value.
                    else:
                        values_to_select_from = list(set(curr_gene_space) - set([offspring[offspring_idx, gene_idx]]))

                        if len(values_to_select_from) == 0:
                            value_from_space = offspring[offspring_idx, gene_idx]
                        else:
                            value_from_space = random.choice(values_to_select_from)
                        
                        #if motor
                        if gene_idx in motor_comb_idxs():
                            # print("checking motor")
                            battery = offspring[offspring_idx][2+9*n_rotor_max+7*n_depcam_max]
                            S = battery_dict[int(battery)]["S"]
                            if not (value_from_space in available_motor_combs(S)):
                                value_from_space = offspring[offspring_idx, gene_idx]
                                # print("Invalid change")
                        
                        #if battery
                        if gene_idx == 2+9*n_rotor_max+7*n_depcam_max: 
                            # print("changing battery")
                            S = battery_dict[int(value_from_space)]["S"]
                            motors = available_motor_combs(S)
                            for i in range(n_rotor_max):
                                comb_num = int(offspring[offspring_idx][2+9*i])
                                if not (comb_num in motors):
                                    offspring[offspring_idx][2+9*i] = random.choice(motors)
                                    # print(f"changed from motor {comb_num} to {offspring[offspring_idx][2+9*i]}")
                            

            else:
                # Selecting a value randomly from the global gene space in the 'gene_space' attribute.
                if type(ga_instance.gene_space) is dict:
                    # When the gene_space is assigned a dict object, then it specifies the lower and upper limits of all genes in the space.
                    if 'step' in ga_instance.gene_space.keys():
                        value_from_space = np.random.choice(np.arange(start=ga_instance.gene_space['low'],
                                                                            stop=ga_instance.gene_space['high'],
                                                                            step=ga_instance.gene_space['step']),
                                                                size=1)[0]
                    else:
                        value_from_space = np.random.uniform(low=ga_instance.gene_space['low'],
                                                                high=ga_instance.gene_space['high'],
                                                                size=1)[0]
                else:
                    # If the space type is not of type dict, then a value is randomly selected from the gene_space attribute.
                    values_to_select_from = list(set(ga_instance.gene_space) - set([offspring[offspring_idx, gene_idx]]))

                    if len(values_to_select_from) == 0:
                        value_from_space = offspring[offspring_idx, gene_idx]
                    else:
                        value_from_space = random.choice(values_to_select_from)

                        

                # value_from_space = random.choice(ga_instance.gene_space)

            if value_from_space is None:
                # TODO: Return index 0.
                # TODO: Check if this if statement is necessary.
                value_from_space = np.random.uniform(low=range_min, 
                                                        high=range_max, 
                                                        size=1)[0]

            # Assinging the selected value from the space to the gene.
            if ga_instance.gene_type_single == True:
                if not ga_instance.gene_type[1] is None:
                    offspring[offspring_idx, gene_idx] = np.round(ga_instance.gene_type[0](value_from_space),
                                                                        ga_instance.gene_type[1])
                else:
                    offspring[offspring_idx, gene_idx] = ga_instance.gene_type[0](value_from_space)
                
            else:
                if not ga_instance.gene_type[gene_idx][1] is None:
                    offspring[offspring_idx, gene_idx] = np.round(ga_instance.gene_type[gene_idx][0](value_from_space),
                                                                        ga_instance.gene_type[gene_idx][1])

                else:
                    offspring[offspring_idx, gene_idx] = ga_instance.gene_type[gene_idx][0](value_from_space)
            
            if ga_instance.allow_duplicate_genes == False:
                offspring[offspring_idx], _, _ = ga_instance.solve_duplicate_genes_by_space(solution=offspring[offspring_idx],
                                                                                        gene_type=ga_instance.gene_type,
                                                                                        num_trials=10)
            


    return offspring




# class Lineage():
#     def __init__(self):
#         self.lineage = None

#     def track_lineage(self,ga_instance, parents):
#         if type(self.lineage) == type(None):
#             self.lineage = parents
#             print("hehey")
#         else:
#             for parent in parents:
#                 for grandparent in self.lineage:
#                     parent_diff = parent == grandparent
#                     print(parent_diff)
#                     zeros = ga_instance.num_genes - np.count_nonzero(parent_diff)
#                     print(zeros)
#                     if zeros <= (ga_instance.mutation_num_genes):
#                         print("letsgooooo")





#         print(type(parents))

def on_generation(ga_instance):
    print(f"generations Completed: {ga_instance.generations_completed}")




#Generates an initial population with motor-battery constraints enforced by
# one iteration of the mutation operator
def gen_init_pop():
    
    ga_instance = ga.GA(num_generations=num_generations,
                num_parents_mating=num_parents_mating,
                sol_per_pop=sol_per_pop,
                num_genes=num_genes,
                gene_space=gene_space,
                parent_selection_type='tournament',
                fitness_func=fitness_func,
                save_best_solutions=True,
                mutation_type=mutation_by_space_x,
                mutation_num_genes=15,
                crossover_type=None,
                parallel_processing=["process",10],
                keep_elitism=5,
                keep_parents=0,
                K_tournament=4,
                stop_criteria=["reach_95", "saturate_7"],
                on_generation=on_generation,
                random_seed = 123
                )

    init_pop = ga_instance.initial_population
    one_mutation = mutation_by_space_x(init_pop, ga_instance)
    return one_mutation
def run_ga():
    # lineage = Lineage()
    init_pop = gen_init_pop()
    ga_instance = ga.GA(num_generations=num_generations,
                    num_parents_mating=num_parents_mating,
                    sol_per_pop=sol_per_pop,
                    num_genes=num_genes,
                    gene_space=gene_space,
                    initial_population=init_pop,
                    parent_selection_type='tournament',
                    fitness_func=fitness_func,
                    save_best_solutions=True,
                    mutation_type=mutation_by_space_x,
                    mutation_num_genes=15,
                    crossover_type=None,
                    parallel_processing=["process",10],
                    keep_elitism=5,
                    keep_parents=0,
                    K_tournament=4,
                    stop_criteria=["reach_95", "saturate_7"],
                    on_generation=on_generation,
                    random_seed = 123
                    )
                    
    ga_instance.summary()
    ga_instance.run()
    return ga_instance


if __name__ == "__main__":
    run_ga()


