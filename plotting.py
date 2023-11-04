import matplotlib.pyplot as plt
import numpy as np
import GA as ga
import MultiRotorDynamics as MRD
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D, Patch3D, Poly3DCollection



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
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')


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

def plot_MR_design(MR: MRD.MultiRotor):
    fig = plt.figure()
    xlim = ylim = zlim = [-0.2,0.2]
    ax = fig.add_subplot(projection='3d', xlim=xlim, ylim=ylim, zlim=zlim)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    e1 = np.array([1,0,0])
    e2 = np.array([0,1,0])
    e3 = np.array([0,0,1])

    rotors = MR.rotors
    center = np.array([0,0,0])
    frame_scale = 0.04
    for rotor in rotors:
        t_vec = rotor.t_vec
        R = rotor.R
        
        ax.add_line(Line3D([0,t_vec[0]],[0,t_vec[1]],[0,t_vec[2]]))
        ax.add_line(Line3D([t_vec[0],t_vec[0]+R[0,:]@e1*frame_scale],[t_vec[1],t_vec[1]+R[1,:]@e1*frame_scale],[t_vec[2],t_vec[2]+R[2,:]@e1*frame_scale],color='r')) #x
        ax.add_line(Line3D([t_vec[0],t_vec[0]+R[0,:]@e2*frame_scale],[t_vec[1],t_vec[1]+R[1,:]@e2*frame_scale],[t_vec[2],t_vec[2]+R[2,:]@e2*frame_scale],color='r')) #y
        ax.add_line(Line3D([t_vec[0],t_vec[0]-R[0,:]@e1*frame_scale],[t_vec[1],t_vec[1]-R[1,:]@e1*frame_scale],[t_vec[2],t_vec[2]-R[2,:]@e1*frame_scale],color='r')) #x
        ax.add_line(Line3D([t_vec[0],t_vec[0]-R[0,:]@e2*frame_scale],[t_vec[1],t_vec[1]-R[1,:]@e2*frame_scale],[t_vec[2],t_vec[2]-R[2,:]@e2*frame_scale],color='r')) #y
        ax.add_line(Line3D([t_vec[0],t_vec[0]+R[0,:]@e3*frame_scale],[t_vec[1],t_vec[1]+R[1,:]@e3*frame_scale],[t_vec[2],t_vec[2]+R[2,:]@e3*frame_scale],color='g')) #z
    plt.show()


def main():
    solution = [ 5. ,         1.  ,        1.  ,       -0.17657736, -0.04898606,  0.28597891,
  0.12826444,  0.05370799 , 0.24708044 , 0.09460495 , 1.    ,     14.,
  0.02702993, -0.32856819,  0.2405342 ,  0.19644204 ,-0.70234712 ,-0.05881733,
  0.44639218 , 1.,          9.  ,        0.04166297,  0.07263588, -0.09529657,
  0.05682608,  0.96190323, -0.30276031, -0.90695516, -1.  ,       14.,
  0.31664172, -0.17582351,  0.1006812 ,  0.16044082, -0.49560657 , 0.96388775,
 -0.20009998,  1.,          7.    ,      0.23223684 ,-0.07955633 , 0.25194221,
  0.10793308, -0.38462199, -0.6177086,  -0.36193609,  1. ,         1.,
 -0.18331298, -0.12581055 , 0.12151063  ,0.10616513 , 0.37250898, -0.84666089,
 -0.43686302, -1.    ,      7.        ,  0.0281676 ,  0.31705186 ,-0.31305692,
  0.11870161, -0.83601839 , 0.05407107, -0.6632683,   1.   ,      18.,
 -0.37083189,  0.20145357 ,-0.10796069 , 0.16255493, -0.46770887,  0.75701491,
 -0.05781761,  1.   ,      -0.25652178,  0.35000877, -0.1946251,   0.02042994,
 -0.76468714, -0.80020139, -0.1963501 , -0.3108279  , 0.03001285 , 0.36007225,
  0.19385211, -0.97111422 , 0.81601145,  0.92142891]

    MR = ga.load_MR_from_sol(solution)

    plot_MR_design(MR)


if __name__ == "__main__":
    main()
    





