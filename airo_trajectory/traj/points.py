#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import numpy.matlib
# Parameters
sample_time = 0.025                 #seconds
hover_thrust = 0.26
cycles = 5
step_interval = 5

points_matrix = np.array([[-0.5,-0.5,1],[1.5,-0.5,1],[1.5,1.5,1],[-0.5,1.5,1]])

# Trajectory
duration = cycles*np.size(points_matrix,0)*step_interval

traj = np.zeros((int(duration/sample_time+1),11)) #x y z u v w phi theta thrust phi_cmd theta_cmd

t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,3] = 0
traj[:,4] = 0
traj[:,5] = 0
traj[:,6] = 0
traj[:,7] = 0
traj[:,8] = hover_thrust
traj[:,9] = 0
traj[:,10] = 0

for i in range(1,cycles+1):
    for j in range(1,np.size(points_matrix,0)+1):
        traj_start = (i-1)*np.size(points_matrix,0)*step_interval+(j-1)*step_interval
        traj_end = (i-1)*np.size(points_matrix,0)*step_interval+j*step_interval
        traj[int(traj_start/sample_time):int(traj_end/sample_time),0:3] = np.tile(points_matrix[j-1,:],(int(step_interval/sample_time),1))

traj[-1,0:3] = traj[-2,0:3]

# write to txt
np.savetxt('points.txt',traj,fmt='%f')