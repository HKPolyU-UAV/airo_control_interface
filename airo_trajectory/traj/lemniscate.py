#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

# Parameters
sample_time = 1/40                #seconds
duration = 30;                      #seconds

amp = 2
frq = 1

x0 = 0.5
y0 = 0.5
z0 = 1

# Trajectory
traj = np.zeros((int(duration/sample_time+1),6)) #x y z u v w
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = amp*np.cos(t*frq)+x0
traj[:,1] = amp*np.sin(t*frq)*np.cos(t*frq)+y0
traj[:,2] = z0
traj[:,3] = -amp*frq*np.sin(t*frq)
traj[:,4] = amp*frq*np.cos(t*2*frq)
traj[:,5] = 0

# Write to txt
np.savetxt('lemniscate.txt',traj,fmt='%f')