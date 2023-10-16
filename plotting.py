import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

def plot_attitude(rot_vec_history,rot_vec_est_history,delta_t):
    num_data_points = rot_vec_history.shape[1]
    t = np.linspace(0,num_data_points*delta_t,num_data_points)
    ylabels = ["z (radians)","x (radians)","y (radians)"]
    titles = ["yaw","roll","pitch"]
    fig, ax = plt.subplots(3,1)
    for i in range(rot_vec_history.shape[0]):
        
        ax[i].plot(t,rot_vec_history[i])
        ax[i].plot(t,rot_vec_est_history[i])

        ax[i].set(xlabel='time (s)', ylabel=ylabels[i],
        title=titles[i])
        ax[i].grid()

        fig.savefig("data/test.png")
    plt.show()

def plot_position_3d(t_vec_history, obst_wf):
    X, Y, Z  = t_vec_history[0],t_vec_history[1],t_vec_history[2]
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(X,Y,Z, label = "position")  # Plot contour curves
    radius = 0.2
    height_tree = 10

    for i in range(obst_wf.shape[1]):
        x_grid,y_grid,z_grid = data_for_cylinder_along_z(obst_wf[0,i],obst_wf[1,i],radius,height_tree)
        ax.plot_surface(x_grid,y_grid,z_grid)

    ax.legend()
    ax.set_xlim(0,5)
    ax.set_ylim(0,5)
    ax.set_zlim(0,10)


    plt.show()

def plot_position_2d(t_vec_history,delta_t):
    num_data_points = t_vec_history.shape[1]
    t = np.linspace(0,num_data_points*delta_t,num_data_points)
    ylabels = ["x (meters)","y (meters)","z (meters)"]
    titles = ["x","y","z"]
    fig, ax = plt.subplots(3,1)
    for i in range(t_vec_history.shape[0]):
        
        ax[i].plot(t,t_vec_history[i])

        ax[i].set(xlabel='time (s)', ylabel=ylabels[i],
        title=titles[i])
        ax[i].grid()

        fig.savefig("data/test.png")
    plt.show()


# def plot_obst_3d(obst_wf):
#     fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')

    





