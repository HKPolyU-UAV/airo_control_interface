#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

# Parameters
sample_time = 1/40             # seconds
duration = 30                  # seconds

r = 2.5                        # m
v = 2.5                          # m/s

# Circle Center
x0 = 0.5                
y0 = 0.5
z0 = 1

# Trajectory
traj = np.zeros((int(duration/sample_time+1),7)) #x y z u v w psi
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = -r*np.cos(t*v/r)+x0
traj[:,1] = -r*np.sin(t*v/r)+y0
traj[:,2] = z0
traj[:,3] = v*np.sin(t*v/r)
traj[:,4] = -v*np.cos(t*v/r)
traj[:,5] = 0
traj[:,6] = np.arctan2(np.sin(t*v/r),np.cos(t*v/r))

# write to txt
np.savetxt('circle.txt',traj,fmt='%f')