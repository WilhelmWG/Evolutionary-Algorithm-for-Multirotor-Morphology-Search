import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

def plot_attitude(rot_vec_history,delta_t):
    num_data_points = rot_vec_history.shape[1]
    t = np.linspace(0,num_data_points*delta_t,num_data_points)
    ylabels = ["z (radians)","x (radians)","y (radians)"]
    titles = ["yaw","roll","pitch"]
    fig, ax = plt.subplots(3,1)
    for i in range(rot_vec_history.shape[0]):
        
        ax[i].plot(t,rot_vec_history[i])

        ax[i].set(xlabel='time (s)', ylabel=ylabels[i],
        title=titles[i])
        ax[i].grid()

        fig.savefig("data/test.png")
    plt.show()

def plot_position_3d(t_vec_history, obst_wf):
    X, Y, Z  = t_vec_history[0],t_vec_history[1],t_vec_history[2]
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(X,Y,Z, label = "position")  # Plot contour curves
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = 0.4 * np.outer(np.cos(u), np.sin(v))
    y = 0.4 * np.outer(np.sin(u), np.sin(v))
    z = 0.4 * np.outer(np.ones(np.size(u)), np.cos(v))
    for i in range(obst_wf.shape[1]):
        ax.plot_surface(x+obst_wf[0,i],y+obst_wf[1,i],z+obst_wf[2,i])
    ax.legend()
    # ax.set_xlim(0,5)
    # ax.set_zlim(0,5)
    # ax.set_ylim(0,5)


    plt.show()

# def plot_obst_3d(obst_wf):
#     fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')

    





