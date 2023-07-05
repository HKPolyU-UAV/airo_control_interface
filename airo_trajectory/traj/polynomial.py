
#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

def get_polynomial(poly_coeff: np.ndarray, t: np.ndarray, axis):
    poly = np.zeros((t.size))
    for i in range(poly_coeff[0].size):
        poly += np.power(t, i) * poly_coeff[axis][i]
    return poly

def generate_trajectory(poly_coeff: np.ndarray, duration, sample_time):
    t = np.arange(0,duration,sample_time)

    traj = np.zeros((t.size,3)) #x y z u v w du dv dw
    
    traj[:,0] = get_polynomial(poly_coeff,t,0)
    traj[:,1] = get_polynomial(poly_coeff,t,1)
    traj[:,2] = get_polynomial(poly_coeff,t,2)

    return traj

# Parameters
sample_time = 1/40                  #seconds

poly_coeff_1 = np.zeros((3,6))
poly_coeff_1[0] = [-0.5,  5.40862e-16, -3.62174e-16,      0.11806,   -0.0281637,   0.00175738]
poly_coeff_1[1] = [-0.5,  3.04479e-16, -2.61388e-17,   -0.0194389,   0.00865493,  -0.00071959]
poly_coeff_1[2] = [0.75,  1.73147e-16, -5.61672e-17,    0.0094023, -0.000935284,  1.96301e-06]
poly_coeff_2 = np.zeros((3,6))
poly_coeff_2[0] = [2,     0.464305,    -0.235702,   -0.0283839,    0.0122392, -0.000830817 ]
poly_coeff_2[1] = [0,     0.524294,     0.130224,   -0.0123927,  -0.00788871,    0.0008133]
poly_coeff_2[2] = [1.25,     0.237056,    0.0129615,   -0.0073847, -0.000890154,  0.000143162]
poly_coeff_3 = np.zeros((3,6))
poly_coeff_3[0] = [0,    -0.532436,     0.131747,    0.0129185,  -0.00800068,   0.00072162]
poly_coeff_3[1] = [2,    -0.447395,    -0.233852,    0.0269334,    0.0119244,  -0.00167716]
poly_coeff_3[2] = [1.75,       -0.171,   -0.0561815,   0.00925248,   0.00259748, -0.000406968]

poly_time = np.array([4.59808,4.87228,4.65754])

# Trajectory
traj_1 = generate_trajectory(poly_coeff_1, poly_time[0], sample_time)
traj_2 = generate_trajectory(poly_coeff_2, poly_time[1], sample_time)
traj_3 = generate_trajectory(poly_coeff_3, poly_time[2], sample_time)

traj = np.concatenate((traj_1, traj_2, traj_3), axis=0)

# # Write to txt
np.savetxt('polynomial.txt',traj,fmt='%f')
