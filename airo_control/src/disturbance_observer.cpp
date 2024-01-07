#include "airo_control/disturbance_observer.h"

DISTURBANCE_OBSERVER::DISTURBANCE_OBSERVER(ros::NodeHandle& nh,const double& HOVER_THRUST){
    // ROS Parameters
    hover_thrust = HOVER_THRUST;
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_VEL);
    nh.getParam("airo_control_node/observer/r_pos",R_ATT);
    nh.getParam("airo_control_node/observer/r_pos",R_CONTROL);
    nh.getParam("airo_control_node/observer/r_pos",Q_POS);
    nh.getParam("airo_control_node/observer/r_pos",Q_VEL);
    nh.getParam("airo_control_node/observer/r_pos",Q_ATT);
    nh.getParam("airo_control_node/observer/r_pos",Q_DISTURBANCE);

    // Weights
    // Q_noise, R_noise, P0 init
    Q_noise = Q_cov.asDiagonal();
    R_noise = Eigen::MatrixXd::Identity(m,m)*(pow(dt,4)/4);
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
    Eigen::Vector3d force_disturbance;
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
    P_pred = F * esti_P * F.transpose() + noise_Q;      // predict covariance at time k+1|k

    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                     // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred);                                 // predict measurement at time k+1
    y_err = meas_y - y_pred;                            // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                      // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred * (MatrixXd::Identity(n, n) - Kal * H).transpose() + Kal*noise_R*Kal.transpose(); // correct covariance estimate

}