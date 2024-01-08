#include "airo_control/disturbance_observer.h"

DISTURBANCE_OBSERVER::DISTURBANCE_OBSERVER(ros::NodeHandle& nh,const double& HOVER_THRUST){
    // ROS Parameters
    hover_thrust = HOVER_THRUST;
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_vel",R_VEL);
    nh.getParam("airo_control_node/observer/r_att",R_ATT);
    nh.getParam("airo_control_node/observer/r_control",R_CONTROL);
    nh.getParam("airo_control_node/observer/q_pos",Q_POS);
    nh.getParam("airo_control_node/observer/q_vel",Q_VEL);
    nh.getParam("airo_control_node/observer/q_att",Q_ATT);
    nh.getParam("airo_control_node/observer/q_disturbance",Q_DISTURBANCE);
    nh.getParam("airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);


    // Weights
    // Q_noise, R_noise, P0 init
    Q_cov << Q_POS,Q_POS,Q_POS,Q_VEL,Q_VEL,Q_VEL,Q_ATT,Q_ATT,Q_ATT,
                Q_DISTURBANCE,Q_DISTURBANCE,Q_DISTURBANCE;
    Q_noise = Q_cov.asDiagonal();
    R_cov << R_POS,R_POS,R_POS,R_VEL,R_VEL,R_VEL,R_ATT,R_ATT,R_ATT,
                R_CONTROL,R_CONTROL,R_CONTROL;
    R_noise = R_cov.asDiagonal();
    P0 = Eigen::MatrixXd::Identity(m,m);
    esti_P = P0;
    
}

Eigen::Vector3d DISTURBANCE_OBSERVER::observe(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist,const mavros_msgs::AttitudeTarget attitude_target){
    // x,y,z,u,v,w in measurement and system states
    measurement_states.x = pose.pose.position.x; 
    measurement_states.y = pose.pose.position.y;
    measurement_states.z = pose.pose.position.z;
    measurement_states.u = twist.twist.linear.x;   
    measurement_states.v = twist.twist.linear.y; 
    measurement_states.w = twist.twist.linear.z; 
   
    system_states.x = pose.pose.position.x;
    system_states.y = pose.pose.position.y;
    system_states.z = pose.pose.position.z;
    system_states.u = twist.twist.linear.x;   
    system_states.v = twist.twist.linear.y; 
    system_states.w = twist.twist.linear.z;

    // phi,theta,psi in measurement and system states
    current_euler = BASE_CONTROLLER::q2rpy(pose.pose.orientation);
    measurement_states.phi = current_euler.x();
    measurement_states.theta = current_euler.y();
    measurement_states.psi = current_euler.z();

    system_states.phi = current_euler.x();
    system_states.theta = current_euler.y();
    system_states.psi = current_euler.z();

    // disturbances in system state
    Eigen::Vector3Stamped force_disturbance;
    force_disturbance.x() = system_states.disturbance_x;
    force_disturbance.y() = system_states.disturbance_y;
    force_disturbance.z() = system_states.disturbance_z;

    // U1(thrust) in measurement state
    measurement_states.thrust_x = attitude_target.thrust;
    measurement_states.thrust_y = attitude_target.thrust;
    measurement_states.thrust_z = attitude_target.thrust;

    return force_disturbance;
}

void DISTURBANCE_OBSERVER::EKF(){
    // Get input u and measurment y
    input_u << measurement_states.thrust_x, measurement_states.thrust_y, measurement_states.thrust_z;
    
    meas_y << measurement_states.x, measurement_states.y, measurement_states.z,
                measurement_states.u, measurement_states.v, measurement_states.w,
                measurement_states.phi, measurement_states.theta, measurement_states.psi,
                force_disturbance.x(), force_disturbance.y(), force_disturbance.z();
    
    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, input_u);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, input_u);                       // predict state at time k+1|k
    P_pred = F * esti_P * F.transpose() + Q_noise;      // predict covariance at time k+1|k

    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                     // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred);                                 // predict measurement at time k+1
    y_err = meas_y - y_pred;                            // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + R_noise).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                      // correct state estimate
    esti_P = (Eigen::MatrixXd::Identity(m, m) - Kal * H) * P_pred * (Eigen::MatrixXd::Identity(m, m) - Kal * H).transpose() + Kal*R_noise*Kal.transpose(); // correct covariance estimate

    // Update disturbance_x in system state
    force_disturbance.x() = esti_x(9);           
    force_disturbance.y() = esti_x(10);
    force_disturbance.z() = esti_x(11);

    std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "disturbance_x: "<<force_disturbance.x()<<"disturbance_y: "<<force_disturbance.y()<<"disturbance_z: "<<force_disturbance.z()<<std::endl;
}

// 4th order RK for integration
Eigen::MatrixXd OBSERVER_EKF::RK4(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    Eigen::Matrix<double,12,1> k1;
    Eigen::Matrix<double,12,1> k2;
    Eigen::Matrix<double,12,1> k3;
    Eigen::Matrix<double,12,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
Eigen::MatrixXd OBSERVER_EKF::f(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    // Define system dynamics
    Eigen::Matrix<double,12,1> xdot;    
    xdot << x(0),x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),  // x,y,z,u,v,w,phi,theta,psi
            0,0,0;                                         // Disturbance_x, disturbance_y, disturbance_z in du, dv, dw
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
Eigen::MatrixXd OBSERVER_EKF::h(Eigen::MatrixXd x)
{
    // Define measurement model
    Eigen::Matrix<double,12,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),  // x,y,z,u,v,w,phi,theta,psi
        (measurement_states.thrust_x-x(9))*(HOVER_THRUST)/((g)*(cos(x(6))*sin(x(7)*cos(x(8))+sin(x(6))*sin(x(8))))),   // thrust for du, x(11) = disturbance_x    
        (measurement_states.thrust_y-x(10))*(HOVER_THRUST)/((g)*(cos(x(6))*sin(x(7))*sin(x(8))-sin(x(6))*cos(x(8)))),   // thrust for dv, x(12) = disturbance_y
        (measurement_states.thrust_z-x(11)+g)*(HOVER_THRUST)/((g)*(cos(x(6))*cos(x(7))));                               // thrust for dw, x(13) = disturbance_z
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
Eigen::MatrixXd OBSERVER_EKF::compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    // Define Jacobian of system dynamics
    Eigen::Matrix<double,12,12> F;
    double d = 1e-6;                    // finite difference step size
    Eigen::VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        Eigen::VectorXd x1 = x;
        x1(i) += d;
        Eigen::VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
Eigen::MatrixXd OBSERVER_EKF::compute_jacobian_H(Eigen::MatrixXd x)
{
    // Define Jacobian of measurement model
    Eigen::Matrix<double,12,12> H;
    double d = 1e-6;                    // finite difference step size
    Eigen::VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        Eigen::VectorXd x1 = x;
        x1(i) += d;
        Eigen::VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}