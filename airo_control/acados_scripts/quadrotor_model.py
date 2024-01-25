from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos
import numpy as np
import math
from scipy.linalg import block_diag

def export_quadrotor_model() -> AcadosModel:

    model_name = 'quadrotor'

    # constent parameters
    g = 9.80665                       # gravity constant [m/s^2]
    
    # states
    x = SX.sym('x')                 # earth position x
    y = SX.sym('y')                 # earth position y
    z = SX.sym('z')                 # earth position z
    u = SX.sym('u')                 # earth velocity x
    v = SX.sym('v')                 # earth velocity y
    w = SX.sym('w')                 # earth velocity z
    phi = SX.sym('phi')             # roll angle phi
    theta = SX.sym('theta')         # pitch angle
    sym_x = vertcat(x,y,z,u,v,w,phi,theta)
    
    # controls
    thrust = SX.sym('thrust')       # thrust command
    phi_cmd = SX.sym('phi_cmd')     # roll angle command
    theta_cmd = SX.sym('theta_cmd') # pitch angle command
    sym_u = vertcat(thrust,phi_cmd,theta_cmd)

    # parameters
    hover_thrust = SX.sym('hover_thrust')
    tau_phi = SX.sym('tau_phi')
    tau_theta = SX.sym('tau_theta')
    psi = SX.sym('psi')             # yaw angle
    
    delta_x = SX.sym('delta_x')
    delta_y = SX.sym('delta_y')
    delta_z = SX.sym('delta_z')
    sym_p = vertcat(hover_thrust,tau_phi,tau_theta,psi,delta_x,delta_y,delta_z)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    z_dot = SX.sym('z_dot')
    u_dot = SX.sym('u_dot')
    v_dot = SX.sym('v_dot')
    w_dot = SX.sym('w_dot')
    phi_dot = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot')
    sym_xdot = vertcat(x_dot,y_dot,z_dot,u_dot,v_dot,w_dot,phi_dot,theta_dot)

    # dynamics
    dx = u
    dy = v
    dz = w
    du = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * thrust/hover_thrust*g + delta_x
    dv = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * thrust/hover_thrust*g + delta_y
    dw = -g + cos(theta) * cos(phi) * thrust/hover_thrust*g + delta_z
    dphi = (phi_cmd - phi) / tau_phi
    dtheta = (theta_cmd - theta) / tau_theta
    f_expl = vertcat(dx,dy,dz,du,dv,dw,dphi,dtheta)

    f_impl = sym_xdot - f_expl

    # nonlinear least sqares
    cost_y_expr = vertcat(sym_x, sym_u)
    #W = block_diag(W_x, W_u)
    
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u
    model.p = sym_p
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = sym_x
    model.name = model_name
    
    return model

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_quadrotor_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    nparam = model.p.size()[0]

    # Set prediction size
    Tf = 1.0    # Prediction horizon (seconds)
    N = 20      # Prediction steps
    ocp.dims.N = N

    # set parameters
    ocp.parameter_values = np.zeros((nparam, ))

    # set cost
    W_x = np.diag([200, 200, 200, 30, 30, 30, 10, 10])    #Q_mat
    W_u = np.diag([5000, 2000, 2000])                  #R_mat  
    W = block_diag(W_x, W_u)
    ocp.cost.W_e = W_x
    ocp.cost.W = W

    # V_x = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    # V_xe = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    # V_u = np.diag([1.0, 1.0, 1.0])
    # ocp.cost.Vx = V_x
    # ocp.cost.Vx_e = V_xe
    # ocp.cost.Vu = V_u

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    #ocp.cost.cost_type = 'EXTERNAL'
    #ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # set constraints
    u_min = np.array([0, -math.pi/3, -math.pi/3])
    u_max = np.array([1, math.pi/3, math.pi/3])
    x_min = np.array([-math.pi/3,-math.pi/3])
    x_max = np.array([math.pi/3,math.pi/3])
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0,1,2])
    ocp.constraints.lbx = x_min
    ocp.constraints.ubx = x_max
    ocp.constraints.idxbx = np.array([6,7])
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # reference trajectory (will be overwritten later)
    x_ref = np.zeros(nx)
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

    # simX = np.ndarray((N+1, nx))
    # simU = np.ndarray((N, nu))

    # status = ocp_solver.solve()
    # ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    # if status != 0:
    #     raise Exception(f'acados returned status {status}.')

if __name__ == '__main__':
    main()