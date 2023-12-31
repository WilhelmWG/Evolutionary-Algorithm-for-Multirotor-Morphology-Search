import matplotlib.pyplot as plt
import numpy as np
import GA as ga
import MultiRotorDynamics as MRD
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D, Patch3D, Poly3DCollection, Text3D


# from https://stackoverflow.com/questions/26989131/add-cylinder-to-plot
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
    ax = plt.figure().add_subplot(projection='3d')
    for t in t_vec_history:
        X, Y, Z  = t[0],t[1],t[2]
        
        ax.plot(X,Y,Z, label = "position")  # Plot contour curves
        ax.set_xlim(0,5)
        ax.set_ylim(0,5)
        ax.set_zlim(0,10)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
    
    # radius = 0.2
    # height_tree = 10

    # for i in range(obst_wf.shape[1]):
    #     x_grid,y_grid,z_grid = data_for_cylinder_along_z(obst_wf[0,i],obst_wf[1,i],radius,height_tree)
    #     ax.plot_surface(x_grid,y_grid,z_grid)
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


def plot_position_vs_ref_and_errors(t_vec_history,t_vec_desired,Psi,delta_t):
    num_data_points = t_vec_history.shape[1]
    t = np.linspace(0,num_data_points*delta_t,num_data_points)
    ylabels = ["x (meters)","y (meters)","z (meters)","e_x", "Psi"]
    titles = ["x","y","z","squared positional error", "Orientation error"]
    plt.rcParams['font.size'] = 14
    fig, ax = plt.subplots(1,5)

    for i in range(t_vec_history.shape[0]):
        
        ax[i].plot(t[1:],t_vec_history[i,1:num_data_points])
        ax[i].plot(t,t_vec_desired[i,:num_data_points])
        ax[i].set(xlabel='time (s)', ylabel=ylabels[i],
        title=titles[i])

        
        ax[i].grid()

        fig.savefig("data/test.png")

    pos_err_sqrd = np.linalg.norm(t_vec_history - t_vec_desired[:,:num_data_points],axis = 0)**2

    ax[3].plot(t[1:],pos_err_sqrd[1:num_data_points])
    ax[3].set(xlabel='time (s)', ylabel=ylabels[3], title=titles[3])
    ax[3].grid()

    ax[4].plot(t[:-1],Psi)
    ax[4].set(xlabel='time (s)', ylabel=ylabels[4], title=titles[4])
    ax[4].grid()
    plt.show()    

def plot_MR_design(MR: MRD.MultiRotor):
    fig = plt.figure()
    xlim = ylim = zlim = [-0.2,0.2]
    ax = fig.add_subplot(projection='3d', xlim=xlim, ylim=ylim, zlim=zlim)
    ax.set_xlabel('x',size=18)
    ax.set_ylabel('y',size=18)
    ax.set_zlabel('z',size=18)
    
    e1 = np.array([1,0,0])
    e2 = np.array([0,1,0])
    e3 = np.array([0,0,1])

    rotors = MR.rotors
    frame_scale = 0.06
    for rotor in rotors:
        t_vec = rotor.t_vec
        R = rotor.R
        if rotor.sigma == -1:
            color = "m"
        else:
            color = "r"
        ax.add_line(Line3D([0,t_vec[0]],[0,t_vec[1]],[0,t_vec[2]]))
        ax.add_line(Line3D([t_vec[0],t_vec[0]+R[0,:]@e1*frame_scale],[t_vec[1],t_vec[1]+R[1,:]@e1*frame_scale],[t_vec[2],t_vec[2]+R[2,:]@e1*frame_scale],color=color)) #x
        ax.add_line(Line3D([t_vec[0],t_vec[0]+R[0,:]@e2*frame_scale],[t_vec[1],t_vec[1]+R[1,:]@e2*frame_scale],[t_vec[2],t_vec[2]+R[2,:]@e2*frame_scale],color=color)) #y
        ax.add_line(Line3D([t_vec[0],t_vec[0]-R[0,:]@e1*frame_scale],[t_vec[1],t_vec[1]-R[1,:]@e1*frame_scale],[t_vec[2],t_vec[2]-R[2,:]@e1*frame_scale],color=color)) #x
        ax.add_line(Line3D([t_vec[0],t_vec[0]-R[0,:]@e2*frame_scale],[t_vec[1],t_vec[1]-R[1,:]@e2*frame_scale],[t_vec[2],t_vec[2]-R[2,:]@e2*frame_scale],color=color)) #y
        ax.add_line(Line3D([t_vec[0],t_vec[0]+R[0,:]@e3*frame_scale],[t_vec[1],t_vec[1]+R[1,:]@e3*frame_scale],[t_vec[2],t_vec[2]+R[2,:]@e3*frame_scale],color='g')) #z

        ax.text(x=t_vec[0],y=t_vec[1],z=t_vec[2], s=rotor.comb_num)
    


    ax.add_line(Line3D([0,frame_scale],[0,0],[0,0],color = "r")) #x 
    ax.add_line(Line3D([0,0],[0,frame_scale],[0,0],color = "r")) #y 
    ax.add_line(Line3D([0,0],[0,0],[0,frame_scale],color = "r")) #z 
    ax.text(x=frame_scale,y=0,z=0, s="x",size = 14)
    ax.text(x=0,y=frame_scale,z=0, s="y",size = 14)
    ax.text(x=0,y=0,z=frame_scale, s="z",size = 14)

    # frame_scale = 0.03
    #Makes a Cube at centroid representing the battery
    # ax.add_line(Line3D([-frame_scale,frame_scale],[frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,frame_scale] ,[-frame_scale,-frame_scale],[frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,frame_scale] ,[frame_scale,frame_scale],[-frame_scale,-frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,frame_scale] ,[-frame_scale,-frame_scale],[-frame_scale,-frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,frame_scale] ,[frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([frame_scale,frame_scale]  ,[-frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,-frame_scale],[-frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([frame_scale,frame_scale],[-frame_scale,frame_scale],[-frame_scale,-frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,-frame_scale],[-frame_scale,frame_scale],[-frame_scale,-frame_scale],color="r"))
    # ax.add_line(Line3D([frame_scale,frame_scale],[frame_scale,frame_scale],[-frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,-frame_scale],[frame_scale,frame_scale],[-frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([frame_scale,frame_scale],[-frame_scale,-frame_scale],[-frame_scale,frame_scale],color="r"))
    # ax.add_line(Line3D([-frame_scale,-frame_scale],[-frame_scale,-frame_scale],[-frame_scale,frame_scale],color="r"))
    ax.text(x=0,y=0,z=0, s=MR.Battery.name)
    plt.show()


def main():
    solution = [ 7.00000000e+00,  1.00000000e+00,  0.00000000e+00,  1.60222696e-01,
  1.17039034e+00, -2.39584093e-01,  3.25137741e-02, -3.49577860e-01,
 -9.79637681e-01,  7.55864155e-01, -1.00000000e+00,  8.00000000e+00,
 -7.94624139e-01,  8.99829575e-01, -3.80054604e-01,  1.59823264e-01,
  5.21282842e-01, -9.18911729e-01, -1.78166398e-01,  1.00000000e+00,
  5.00000000e+00, -3.10593790e-01, -7.01029906e-01,  6.29998326e-01,
  2.27722371e-01,  1.89230314e-01, -1.48166375e-01, -6.08779614e-01,
 -1.00000000e+00,  1.50000000e+01, -9.61443471e-01,  5.37073668e-01,
 -4.33583246e-02,  1.95558375e-01,  8.36060735e-01,  2.73730230e-01,
 -8.69716683e-01, -1.00000000e+00,  1.30000000e+01,  5.27348997e-01,
 -1.50794546e+00,  1.42482558e+00,  7.92364395e-02,  8.04315033e-02,
  2.09897668e-01,  1.40626404e-01,  1.00000000e+00,  6.00000000e+00,
 -1.14860479e+00,  9.60888187e-01,  6.04563372e-01,  1.83537234e-01,
 -6.55632513e-01,  5.11417920e-01,  2.93314760e-01, -1.00000000e+00,
  1.20000000e+01,  2.56092939e-01, -1.33689897e-01, -1.05464327e+00,
  5.68021414e-02,  6.56371639e-01, -5.58359282e-01,  8.17419763e-01,
 -1.00000000e+00,  2.00000000e+00, -1.09144123e+00,  5.25968898e-01,
 -2.80562492e-01,  1.87425752e-01, -9.11286935e-01, -3.11468547e-01,
 -6.18834575e-01,  1.00000000e+00, -1.20341856e+00, -1.47886911e+00,
 -1.44405647e+00,  2.10807114e-01, -5.17359165e-03, -3.62843804e-01,
 -8.24956358e-01, -1.02490805e+00, -7.20114827e-01, -4.21699075e-02,
  2.81007097e-01, -3.20826158e-01, -2.86393480e-01,  4.27268184e-01,
  7.00000000e+00,  9.70632517e+00,  2.82489478e+00,  1.56112257e+00,
  1.93387098e-01]
    MR = ga.load_MR_from_sol(solution)

    plot_MR_design(MR)


if __name__ == "__main__":
    main()
    





