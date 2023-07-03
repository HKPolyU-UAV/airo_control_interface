
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
poly_coeff_1[0] = [0,  4.68084e-15, -1.82008e-16,     0.172267,    -0.026382,   0.00107469]
poly_coeff_1[1] = [0,  1.47423e-15,  1.49699e-16,      0.19362,   -0.0363363,   0.00170035]
poly_coeff_1[2] = [2,  5.29868e-16,  7.40581e-18,    0.0383556,  -0.00677834,  0.000305908]
poly_coeff_2 = np.zeros((3,6))
poly_coeff_2[0] = [13,      2.34381,    -0.399886,   -0.0549236,   0.00925262, -0.000323964]
poly_coeff_2[1] = [8,    -0.401056,    -0.776981,    -0.022468,     0.020044,  -0.00109514]
poly_coeff_2[2] = [4,     0.111185,    -0.133348,  -0.00691676,   0.00336496, -0.000171034]
poly_coeff_3 = np.zeros((3,6))
poly_coeff_3[0] = [5,      -2.2332,     0.190171,    0.0316587,  -0.00393535, -2.72622e-05]
poly_coeff_3[1] = [-10,      1.68896,      0.73587,   -0.0956278,   -0.0245369,   0.00334188]
poly_coeff_3[2] = [1,    0.0708688,     0.112983,   -0.0107031,  -0.00359749,  0.000450212]

poly_time = np.array([6.6316,8.14162,5.24166])

# Trajectory
traj_1 = generate_trajectory(poly_coeff_1, poly_time[0], sample_time)
traj_2 = generate_trajectory(poly_coeff_2, poly_time[1], sample_time)
traj_3 = generate_trajectory(poly_coeff_3, poly_time[2], sample_time)

traj = np.concatenate((traj_1, traj_2, traj_3), axis=0)

# # Write to txt
np.savetxt('polynomial.txt',traj,fmt='%f')
