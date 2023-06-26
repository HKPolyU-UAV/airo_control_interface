#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

# Parameters
sample_time = 0.025                 #seconds
duration = 60;                      #seconds
hover_thrust = 0.26
amp = 2
frq = 2

x0 = 0
y0 = 0
z0 = 1

# Trajectory
traj = np.zeros((int(duration/sample_time+1),11)) #x y z u v w phi theta thrust phi_cmd theta_cmd
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = amp*np.cos(t*frq)+x0
traj[:,1] = amp*np.sin(t*frq)*np.cos(t*frq)+y0
traj[:,2] = z0
traj[:,3] = -amp*frq*np.sin(t*frq)
traj[:,4] = amp*frq*np.cos(t*2*frq)
traj[:,5] = 0
traj[:,6] = 0
traj[:,7] = 0
traj[:,8] = hover_thrust
traj[:,9] = 0
traj[:,10] = 0

# write to txt
np.savetxt('lemniscate.txt',traj,fmt='%f')