import control as ct
import numpy as np
import GA as ga
import MultiRotorDynamics as MRD
import plotting as plt





def main():
    ga_instance = ga.run_ga()
    best_sol = ga_instance.best_solutions
    
    ga_instance.plot_fitness()
    ga_instance.plot_genes(graph_type = "boxplot",solutions="best")
    ga_instance.plot_genes(graph_type = "histogram",solutions="best")
    MR = ga.load_MR_from_sol(best_sol[50])
    plt.plot_MR_design(MR)
    # plt.plot_attitude(MR.rot_vec_history, MR.IMU.rot_vec_est_history, ga.delta_t)
    # plt.plot_position_2d(MR.t_vec_history,ga.delta_t)
    # plt.plot_position_3d(MR.t_vec_history,ga.obst_wf)


if __name__ == "__main__":
    main()