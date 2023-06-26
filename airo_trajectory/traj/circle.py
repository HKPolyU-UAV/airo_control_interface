#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import math

# Parameters
sample_time = 0.025             # seconds
duration = 60                   # seconds
hover_thrust = 0.56
r = 1.5
v = 2

x0 = 0.5                        
y0 = 0.5
z0 = 1

# trajectory
traj = np.zeros((int(duration/sample_time+1),13)) #x y z u v w phi theta psi thrust phi_cmd theta_cmd psi_cmd
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = -r*np.cos(t*v/r)+x0
traj[:,1] = -r*np.sin(t*v/r)+y0
traj[:,2] = z0
traj[:,3] = v*np.sin(t*v/r)
traj[:,4] = -v*np.cos(t*v/r)
traj[:,5] = 0
traj[:,6] = 0
traj[:,7] = 0
traj[:,8] = hover_thrust
traj[:,9] = 0
traj[:,10] = 0



#print(np.size(traj))


# write to txt
np.savetxt('circle.txt',traj,fmt='%f')
