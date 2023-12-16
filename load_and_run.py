import numpy as np
import pygad
import GA as ga
import MultiRotorDynamics as MRD
import plotting as plt
from Trajs import x_ds, b1_ds, b3_ds


# ga_instance_path = "data/ga15mutato"
solution_path = "data/bs_randomb1d.txt"
# solution_path = "data/quadrotor.txt"
# solution_path = "data/OctorotorHorizontal.txt"
# solution_path = "data/tilted_hexa_rotor.txt"
trajectory_number =19
random_init = False

def main():
    f = open(solution_path,"r")
    line = f.read()
    solution = np.fromstring(line[2:-2],sep = " ")
    # ga_instance = pygad.load(ga_instance_path)

    MR = ga.load_MR_from_sol(solution)
    for i in range(trajectory_number):
        MR.next_trajectory(random_init)
    MR.t_vec = [-0.3,-0.3,0]
    MR.simulate(ga.max_time,ga.delta_t,ga.obst_wf)
    plt.plot_MR_design(MR)
    plt.plot_position_vs_ref_and_errors(MR.t_vec_history,MR.Controller.TP.x_d,MR.rot_err_history,ga.delta_t)
    fitness = ga.fitness_func(0,solution,0)
    print(MR.total_mass)
    

if __name__ == "__main__":
    main()