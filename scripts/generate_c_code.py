from acados_template import AcadosOcp, AcadosOcpSolver
from quadrotor import export_quadrotor_model
import numpy as np
import casadi
#from utils import plot_pendulum
import math
from scipy.linalg import block_diag

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_quadrotor_model()
    ocp.model = model

    Tf = 1.0
    
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    
    N = 40

    # set dimensions
    ocp.dims.N = N

    # set cost
    # W_x = np.diag([120, 120, 120, 10, 10, 10, 10, 10, 1000])    #Q_mat
    # W_u = np.diag([5000, 2000, 2000, 100])                     #R_mat
    W_x = np.diag([120, 120, 120, 10, 10, 10, 10, 10])    #Q_mat
    W_u = np.diag([5000, 2000, 2000])                     #R_mat
    W = block_diag(W_x, W_u)
    ocp.cost.W_e = W_x
    ocp.cost.W = W

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    #ocp.cost.cost_type = 'EXTERNAL'
    #ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # set constraints
    # u_min = np.array([0.3, -math.pi/2, -math.pi/2, 0])
    # u_max = np.array([0.9, math.pi/2, math.pi/2, math.pi*2-0.01])
    u_min = np.array([0, -math.pi/2, -math.pi/2])
    u_max = np.array([1, math.pi/2, math.pi/2])
    x_min = np.array([-math.pi/2,-math.pi/2])
    x_max = np.array([math.pi/2,math.pi/2])
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0,1,2])
    ocp.constraints.lbx = x_min
    ocp.constraints.ubx = x_max
    ocp.constraints.idxbx = np.array([6,7])

    # ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # reference trajectory (will be overwritten later)
    x_ref = np.zeros(nx)
    # ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0, 0.0])))
    ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0])))
    ocp.cost.yref_e = x_ref

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    #ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.qp_solver_cond_N = 5
    #ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP


    # set prediction horizon
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    status = ocp_solver.solve()
    ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
        simX[N,:] = ocp_solver.get(N, "x")

    #plot_pendulum(np.linspace(0, Tf, N+1), Fmax, simU, simX, latexify=False)

if __name__ == '__main__':
    main()