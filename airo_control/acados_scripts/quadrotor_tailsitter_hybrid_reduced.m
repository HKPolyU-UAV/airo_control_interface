function model = quadrotor_tailsitter_hybrid()

import casadi.*
load('network_model_hybrid_reduced.mat');

%% system dimensions
nx = 8;
nu = 3;

%% system parameters

g = 9.81;
hover_thrust = 0.2;
tau_phi = 0.1667;                               % Inner-loop controller time constants
tau_theta = 0.1667;

%% states

x = SX.sym('x');                    % earth position x
y = SX.sym('y');                    % earth position y
z = SX.sym('z');                    % earth position z
u = SX.sym('u');                    % earth velocity x
v = SX.sym('v');                    % earth velocity y
w = SX.sym('w');                    % earth velocity z
phi = SX.sym('phi');                % roll angle phi
theta = SX.sym('theta');            % pitch angle
sym_x = vertcat(x,y,z,u,v,w,phi,theta);

%% controls

thrust = SX.sym('thrust');          % thrust command
phi_cmd = SX.sym('phi_cmd');        % roll angle command
theta_cmd = SX.sym('theta_cmd');    % pitch angle command
sym_u = vertcat(thrust,phi_cmd,theta_cmd);

%% xdot for f_impl

x_dot = SX.sym('x_dot');
y_dot = SX.sym('y_dot');
z_dot = SX.sym('z_dot');
u_dot = SX.sym('u_dot');
v_dot = SX.sym('v_dot');
w_dot = SX.sym('w_dot');
phi_dot = SX.sym('phi_dot');
theta_dot = SX.sym('theta_dot');
sym_xdot = vertcat(x_dot,y_dot,z_dot,u_dot,v_dot,w_dot,phi_dot,theta_dot);

%% network implementation

% Input 1
NN_input = [u;v;w;phi;theta;thrust;];
u_xoffset = net.input.processSettings{1,1}.xmin;
u_gain = net.input.processSettings{1,1}.gain;
u_ymin = repmat(-1,size(u_xoffset,1),1);

% Layer 1
b1 = net.b{1,1};
IW1_1 = net.IW{1,1};

% Layer 2
b2 = net.b{2,1};
LW2_1 = net.LW{2,1};

% Output 1
dot_ymin = repmat(-1,size(b2,1),1);
dot_gain = net.output.processSettings{1,1}.gain;
dot_xoffset = net.output.processSettings{1,1}.xmin;

% Input Mapping
xp = NN_input - u_xoffset;
xp = xp .* u_gain;
xp = xp + u_ymin;

% Layer 1
a1 = repmat(2,size(b1,1),1) ./ (ones(size(b1,1),1) + exp(-2*(b1 + IW1_1*xp))) - ones(size(b1,1),1);

% Layer 2
a2 = repmat(b2,1,1) + LW2_1*a1;

% Output Mapping
NN_output = a2 - dot_ymin;
NN_output = NN_output ./ dot_gain;
NN_output = NN_output + dot_xoffset;

%% dynamics

dx = u;
dy = v;
dz = w;
du = sin(theta) * cos(phi) * thrust/hover_thrust*g + NN_output(1);
dv =-sin(phi) * thrust/hover_thrust*g + NN_output(2);
dw =-g + cos(theta) * cos(phi) * thrust/hover_thrust*g + NN_output(3);
dphi = (phi_cmd - phi) / tau_phi;
dtheta = (theta_cmd - theta) / tau_theta;

expr_f_expl = vertcat(dx,dy,dz,du,dv,dw,dphi,dtheta);
expr_f_impl = expr_f_expl - sym_xdot;

%% constraints

expr_h = sym_u;

%% cost

W_x = diag([60 60 60 10 10 10 10 10]);
W_u = diag([2000 200 300]);
expr_ext_cost_e = sym_x'* W_x * sym_x;
expr_ext_cost = expr_ext_cost_e + sym_u' * W_u * sym_u;
% nonlinear least sqares
cost_expr_y = vertcat(sym_x, sym_u);
W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_x;
model.W_e = W_x;

%% populate structure

model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;

model.cost_expr_y = cost_expr_y;
model.W = W;

end