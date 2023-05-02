from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_quadrotor_model() -> AcadosModel:

    model_name = 'quadrotor'

    # system parameters
    g = 9.81                        # gravity constant [m/s^2]
    hover_thrust = 0.56
    tau_phi = 0.1667                # Inner-loop controller time constants
    tau_theta = 0.1667
    tau_psi = 0.1667

    # states
    x = SX.sym('x')                 # earth position x
    y = SX.sym('y')                 # earth position y
    z = SX.sym('z')                 # earth position z
    u = SX.sym('u')                 # earth velocity x
    v = SX.sym('v')                 # earth velocity y
    w = SX.sym('w')                 # earth velocity z
    phi = SX.sym('phi')             # roll angle phi
    theta = SX.sym('theta')         # pitch angle
    # psi = SX.sym('psi')             # yaw angle
    # sym_x = vertcat(x,y,z,u,v,w,phi,theta,psi)
    sym_x = vertcat(x,y,z,u,v,w,phi,theta)

    # controls
    thrust = SX.sym('thrust')       # thrust command
    phi_cmd = SX.sym('phi_cmd')     # roll angle command
    theta_cmd = SX.sym('theta_cmd') # pitch angle command
    # psi_cmd = SX.sym('psi_cmd')     # yaw angle command
    # sym_u = vertcat(thrust,phi_cmd,theta_cmd,psi_cmd)
    sym_u = vertcat(thrust,phi_cmd,theta_cmd)

    # xdot for f_impl
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    z_dot = SX.sym('z_dot')
    u_dot = SX.sym('u_dot')
    v_dot = SX.sym('v_dot')
    w_dot = SX.sym('w_dot')
    phi_dot = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot')
    # psi_dot = SX.sym('psi_dot')
    # sym_xdot = vertcat(x_dot,y_dot,z_dot,u_dot,v_dot,w_dot,phi_dot,theta_dot,psi_dot)
    sym_xdot = vertcat(x_dot,y_dot,z_dot,u_dot,v_dot,w_dot,phi_dot,theta_dot)

    # dynamics
    dx = u
    dy = v
    dz = w
    # du = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * thrust/hover_thrust*g
    # dv = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * thrust/hover_thrust*g
    # dw = -g + cos(theta) * cos(phi) * thrust/hover_thrust*g
    du = sin(theta) * cos(phi) * thrust/hover_thrust*g
    dv = -sin(phi) * thrust/hover_thrust*g
    dw = -g + cos(theta) * cos(phi) * thrust/hover_thrust*g
    dphi = (phi_cmd - phi) / tau_phi
    dtheta = (theta_cmd - theta) / tau_theta
    # dpsi = (psi_cmd - psi) / tau_psi
    # f_expl = vertcat(dx,dy,dz,du,dv,dw,dphi,dtheta,dpsi)
    f_expl = vertcat(dx,dy,dz,du,dv,dw,dphi,dtheta)

    f_impl = sym_xdot - f_expl


    
    # constraints
    h_expr = sym_u
    
    # cost
    #W_x = np.diag([120, 120, 120, 10, 10, 10, 10, 10])
    #W_u = np.diag([5000, 2000, 2000])

    #expr_ext_cost_e = sym_x.transpose()* W_x * sym_x
    #expr_ext_cost = expr_ext_cost_e + sym_u.transpose() * W_u * sym_u
    

    # nonlinear least sqares
    cost_y_expr = vertcat(sym_x, sym_u)
    #W = block_diag(W_x, W_u)
    
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = sym_xdot
    model.u = sym_u
    model.cost_y_expr = cost_y_expr
    model.cost_y_expr_e = sym_x
    #model.con_h_expr = h_expr
    model.name = model_name
    #model.cost_expr_ext_cost = expr_ext_cost
    #model.cost_expr_ext_cost_e = expr_ext_cost_e 

    return model