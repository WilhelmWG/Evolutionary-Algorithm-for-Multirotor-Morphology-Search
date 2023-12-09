import numpy as np

x_ds = []
b1_ds = []
b3_ds = []

#NoMovement
x_d = lambda t : np.array([0*t,0*t,0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#NoMovementRotate
x_d = lambda t : np.array([0*t,0*t,0*t])
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory1 z+
x_d = lambda t : np.array([0*t,0*t,1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#UPRIGHT TRAJ
#Trajectory1 z-
x_d = lambda t : np.array([0*t,0*t,-1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory2 y+z+
x_d = lambda t : np.array([0*t,1*t,1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory3 x+z+
x_d = lambda t : np.array([1*t,0*t,1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory4 y+z-
x_d = lambda t : np.array([0*t,1*t,-1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory5 x+z-
x_d = lambda t : np.array([1*t,0*t,-1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory2 y+z+
x_d = lambda t : np.array([0*t,1*t,1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory3 x+z+
x_d = lambda t : np.array([1*t,0*t,1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory4 y+z-
x_d = lambda t : np.array([0*t,1*t,-1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory5 x+z-
x_d = lambda t : np.array([1*t,0*t,-1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory5 x+y+z-
x_d = lambda t : np.array([1/np.sqrt(2)*t,1/np.sqrt(2)*t,-1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory4 y+
x_d = lambda t : np.array([0*t,1*t,0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory5 x+
x_d = lambda t : np.array([1*t,0*t,0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
# b3_d = lambda t : np.array([0*t,1/np.sqrt(2)*t/t,1/np.sqrt(2)*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Trajectory5 x+y+
x_d = lambda t : np.array([1/np.sqrt(2)*t,1/np.sqrt(2)*t,0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)


#Trajectory5 x+y+z+
x_d = lambda t : np.array([1/np.sqrt(2)*t,1/np.sqrt(2)*t,1*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)


#TrajectoryLast
x_d = lambda t : np.array([0.4*t,0.4*np.sin(np.pi*t),0.6*np.cos(np.pi*t)])# x_d = lambda t : np.array([0*t,1*t,1*t])#
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#TrajectoryLast
x_d = lambda t : np.array([0.4*np.sin(np.pi*t),0.4*t,0.6*np.cos(np.pi*t)])# x_d = lambda t : np.array([0*t,1*t,1*t])#
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Swivvle TRAJ
x_d = lambda t : np.array([0.5*np.cos(np.pi*t),1*t,0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

x_d = lambda t : np.array([1*t,0.5*np.cos(np.pi*t),0*t])
b1_d = lambda t : np.array([1*t/t,0*t,0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

#Swivvle + Rotation
x_d = lambda t : np.array([0.5*np.cos(np.pi*t),1*t,0*t])
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)

x_d = lambda t : np.array([1*t,0.5*np.cos(np.pi*t),0*t])
b1_d = lambda t : np.array([np.cos(np.pi*t),np.sin(np.pi*t),0*t])
b3_d = lambda t : np.array([0*t,0*t,1*t/t])
x_ds.append(x_d)
b1_ds.append(b1_d)
b3_ds.append(b3_d)
