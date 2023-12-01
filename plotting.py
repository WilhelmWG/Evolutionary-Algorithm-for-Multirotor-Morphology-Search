import matplotlib.pyplot as plt
import numpy as np
import GA as ga
import MultiRotorDynamics as MRD
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D, Patch3D, Poly3DCollection, Text3D



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

        ax.text(x=t_vec[0],y=t_vec[1],z=t_vec[2], s=rotor.name)
    
    frame_scale = 0.03
    #Makes a Cube at centroid representing the battery
    ax.add_line(Line3D([-frame_scale,frame_scale],[frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,frame_scale] ,[-frame_scale,-frame_scale],[frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,frame_scale] ,[frame_scale,frame_scale],[-frame_scale,-frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,frame_scale] ,[-frame_scale,-frame_scale],[-frame_scale,-frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,frame_scale] ,[frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([frame_scale,frame_scale]  ,[-frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,-frame_scale],[-frame_scale,frame_scale],[frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([frame_scale,frame_scale],[-frame_scale,frame_scale],[-frame_scale,-frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,-frame_scale],[-frame_scale,frame_scale],[-frame_scale,-frame_scale],color="r"))
    ax.add_line(Line3D([frame_scale,frame_scale],[frame_scale,frame_scale],[-frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,-frame_scale],[frame_scale,frame_scale],[-frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([frame_scale,frame_scale],[-frame_scale,-frame_scale],[-frame_scale,frame_scale],color="r"))
    ax.add_line(Line3D([-frame_scale,-frame_scale],[-frame_scale,-frame_scale],[-frame_scale,frame_scale],color="r"))
    ax.text(x=0,y=0,z=0, s=MR.Battery.name)
    plt.show()


def main():
    solution = [ 8.00000000e+00,  2.00000000e+00,  6.00000000e+00, -3.69093901e-01,
 -1.20819142e-01 , 3.66339490e-01 , 2.96428037e-01, -7.24466072e-01,
 -9.30388809e-01 , 9.79183604e-01 , 1.00000000e+00,  1.10000000e+01,
 -3.66791215e-01 , 2.45239915e-01 , 1.63871019e-02,  2.52792818e-01,
  8.43591811e-01 ,-1.34597594e-01 ,-7.11511149e-01, -1.00000000e+00,
  2.00000000e+00 ,-2.31348479e-01 ,-2.73012990e-01,  3.50134739e-01,
  2.50599034e-01 , 7.24916397e-01 , 3.01371066e-01,  8.44986890e-02,
  1.00000000e+00 , 1.60000000e+01 , 3.12469227e-01, -3.02853767e-01,
  4.88319451e-03 , 2.02657992e-01 ,-5.94137897e-01, -3.83368027e-01,
  7.89777507e-01 ,-1.00000000e+00 , 1.40000000e+01, -7.22340635e-02,
 -1.86360543e-01 , 3.47962775e-01 , 1.82153593e-01, -1.50883481e-01,
  6.26776542e-01 ,-4.89038234e-01 ,-1.00000000e+00,  1.80000000e+01,
 -2.46294920e-01 ,-3.56599653e-01 ,-2.65117615e-01,  1.07072862e-01,
 -2.08480377e-01 , 5.71605801e-01 ,-6.78326796e-01, -1.00000000e+00,
  8.00000000e+00 ,-2.83398904e-01 , 1.58375829e-01, -2.04778278e-02,
  2.18026024e-01 ,-6.18392058e-01 , 4.46446712e-01, -3.97625223e-01,
  1.00000000e+00 , 1.00000000e+01 ,-1.71052619e-01, -1.38249091e-01,
  1.67055500e-01 , 2.29290011e-01 ,-5.83227230e-01, -7.38932723e-01,
  7.24264031e-01 , 1.00000000e+00 , 2.18932413e-01,  3.66425429e-01,
 -2.42326509e-01 , 1.98992322e-02 , 4.58047618e-01,  2.69576240e-01,
 -8.42350316e-01 ,-3.34621107e-01 , 1.84738643e-01, -1.22516493e-02,
  1.29347909e-01 , 1.70192708e-01 , 6.24127727e-01, -6.39901365e-01,
  1.00000000e+01]
    MR = ga.load_MR_from_sol(solution)

    plot_MR_design(MR)


if __name__ == "__main__":
    main()
    





