#include "airo_control/disturbance_observer.h"

DISTURBANCE_OBSERVER::DISTURBANCE_OBSERVER(ros::NodeHandle& nh,const double& HOVER_THRUST){
    // ROS Parameters
    hover_thrust = HOVER_THRUST;
    nh.getParam("airo_control_node/observer/r_vel_x",R_VEL_X);
    nh.getParam("airo_control_node/observer/r_vel_y",R_VEL_Y);
    nh.getParam("airo_control_node/observer/r_vel_z",R_VEL_Z);
    nh.getParam("airo_control_node/observer/r_control_x",R_CONTROL_X);
    nh.getParam("airo_control_node/observer/r_control_y",R_CONTROL_Y);
    nh.getParam("airo_control_node/observer/r_control_z",R_CONTROL_Z);
    nh.getParam("airo_control_node/observer/q_vel_x",Q_VEL_X);
    nh.getParam("airo_control_node/observer/q_vel_y",Q_VEL_Y);
    nh.getParam("airo_control_node/observer/q_vel_z",Q_VEL_Z);
    nh.getParam("airo_control_node/observer/q_disturbance_x",Q_DISTURBANCE_X);
    nh.getParam("airo_control_node/observer/q_disturbance_y",Q_DISTURBANCE_Y);
    nh.getParam("airo_control_node/observer/q_disturbance_z",Q_DISTURBANCE_Z);
    nh.getParam("airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);

    // Weights
    Q_cov << Q_VEL_X,Q_VEL_Y,Q_VEL_Z,
                Q_DISTURBANCE_X,Q_DISTURBANCE_Y,Q_DISTURBANCE_Z;
    Q_noise = Q_cov.asDiagonal();

    R_cov << R_VEL_X,R_VEL_Y,R_VEL_Z,
                R_CONTROL_X,R_CONTROL_Y,R_CONTROL_Z;
    R_noise = R_cov.asDiagonal();
    
    P0 = Eigen::MatrixXd::Identity(m,m);
    esti_P = P0;
    dt = 1/FSM_FREQUENCY;
 
    esti_x << 0,0,0,0,0,0;
}

Eigen::Vector3d DISTURBANCE_OBSERVER::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Eigen::Vector3d euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.x(), euler.y(), euler.z());
    return euler;
}

Eigen::Matrix3d DISTURBANCE_OBSERVER::q2ROT(const geometry_msgs::Quaternion q)
{
    Eigen::Quaterniond qd(
        q.w,
        q.x,
        q.y,
        q.z
    );

    return qd.toRotationMatrix();
}

Eigen::Vector3d DISTURBANCE_OBSERVER::disturbance_raw(
    const geometry_msgs::AccelStamped& imu_B_msg,
    const mavros_msgs::AttitudeTarget& Thrust_B,
    const geometry_msgs::PoseStamped& pose
){
    Eigen::Vector3d delta_B;
    Eigen::Vector3d imu_accel_B(
        imu_B_msg.accel.linear.x,
        imu_B_msg.accel.linear.y,
        imu_B_msg.accel.linear.z
    );

    Eigen::Matrix3d ROT_I2B = q2ROT(pose.pose.orientation).inverse();

    delta_B = imu_accel_B - Thrust_B.thrust / hover_thrust * Eigen::Vector3d(0.0,0.0,g);

    return delta_B;
}


geometry_msgs::Vector3Stamped DISTURBANCE_OBSERVER::observe(const geometry_msgs::PoseStamped& pose, 
const geometry_msgs::TwistStamped& twist,
const mavros_msgs::AttitudeTarget attitude_target, 
const geometry_msgs::AccelStamped & imu){
    
    // u,v,w in measurement and system states
    measurement_states.u = twist.twist.linear.x;   
    measurement_states.v = twist.twist.linear.y; 
    measurement_states.w = twist.twist.linear.z; 
    system_states.u = twist.twist.linear.x;   
    system_states.v = twist.twist.linear.y; 
    system_states.w = twist.twist.linear.z;
    
    geometry_msgs::Vector3Stamped force_disturbance;
    
    // Linear acceleration 
    accel_body << imu.accel.linear.x, imu.accel.linear.y, imu.accel.linear.z;
    accel_world = q2ROT(pose.pose.orientation) * accel_body;

    accel.x_b = accel_body(0,0);
    accel.y_b = accel_body(1,0);
    accel.z_b = accel_body(2,0);
    accel.x_w = accel_world(0,0);
    accel.y_w = accel_world(1,0);
    accel.z_w = accel_world(2,0);

    // U1(thrust) in measurement state
    thrust_body << attitude_target.thrust, attitude_target.thrust, attitude_target.thrust;
    thrust_world = q2ROT(pose.pose.orientation) * thrust_body;

    // Thrust from MPC output in in body frame
    measurement_states.thrust_x = thrust_body(0,0); 
    measurement_states.thrust_y = thrust_body(1,0);
    measurement_states.thrust_z = thrust_body(2,0);

    // Get measurment y
    meas_y << measurement_states.u, measurement_states.v, measurement_states.w,
                accel.x_b, accel.y_b, accel.z_b;

    // Define Jacobian matrices of system dynamics and measurement model
    Eigen::Matrix<double,6,6> F;     // Jacobian of system dynamics
    Eigen::Matrix<double,6,6> H;     // Jacobian of measurement model

    // Define Kalman gain matrix
    Eigen::Matrix<double,6,6> Kal;

    // Define prediction and update steps
    Eigen::Matrix<double,6,1> x_pred;     // predicted state
    Eigen::Matrix<double,6,6> P_pred;    // predicted covariance
    Eigen::Matrix<double,6,1> y_pred;     // predicted measurement
    Eigen::Matrix<double,6,1> y_err;      // measurement error

    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, input_u);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, input_u);                       // predict state at time k+1|k
    P_pred = F * esti_P * F.transpose() + Q_noise;       // predict covariance at time k+1|k

    
    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                     // compute Jacobian of measurement model at predicted state    
    y_pred = h(x_pred);                                 // predict measurement at time k+1
    y_err = meas_y - y_pred;                            // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + R_noise).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                      // correct state estimate
    esti_P = (Eigen::MatrixXd::Identity(m, m) - Kal * H) * P_pred * (Eigen::MatrixXd::Identity(m, m) - Kal * H).transpose() + Kal*R_noise*Kal.transpose(); // correct covariance estimate

    // Update disturbance_x in system state
    system_states.u = esti_x(0);
    system_states.v = esti_x(1);
    system_states.w = esti_x(2);
    
    system_states.disturbance_x = esti_x(3);    // Unit: ms^-2
    system_states.disturbance_y = esti_x(4);
    system_states.disturbance_z = esti_x(5);

    Eigen::Vector3d Delta_B (system_states.disturbance_x,system_states.disturbance_y,system_states.disturbance_z);

    Eigen::Vector3d Delta_W = q2ROT(pose.pose.orientation)*Delta_B;

    if (Delta_W.x() > 3 || Delta_W.x() < -3) {
    Delta_W.x() = Delta_W.x() > 3 ? 3 : -3;
    }
    if (Delta_W.y() > 3 || Delta_W.y() < -3) {
    Delta_W.y() = Delta_W.y() > 3 ? 3 : -3;
    }
    if (Delta_W.z() > 3 || Delta_W.z() < -3) {
    Delta_W.z() = Delta_W.z() > 3 ? 3 : -3;
    }

    force_disturbance.vector.x = Delta_W.x();
    force_disturbance.vector.y = Delta_W.y();
    force_disturbance.vector.z = Delta_W.z();

    Eigen::Vector3d delta_B = disturbance_raw(
        imu,
        attitude_target,
        pose
    );

    Eigen::Vector3d delta_W = q2ROT(pose.pose.orientation) * delta_B;
    // Store delta_W.x(), delta_W.y(), delta_W.z() in buffer
    delta_x_W_buffer.push_back(delta_W.x()); 
    delta_y_W_buffer.push_back(delta_W.y());
    delta_z_W_buffer.push_back(delta_W.z()); 

    // Remove oldest value if buffer size exceeds window size
    if (delta_x_W_buffer.size() > window_size) delta_x_W_buffer.pop_front();  
    if (delta_y_W_buffer.size() > window_size) delta_y_W_buffer.pop_front(); 
    if (delta_z_W_buffer.size() > window_size) delta_z_W_buffer.pop_front(); 

    // Calculate the mean of delta_W.x(), delta_W.y(), delta_W.z() values in the buffer
    for (const auto& valx : delta_x_W_buffer){
        meanDelta_x_W += valx;
    }
    meanDelta_x_W/= delta_x_W_buffer.size();

    for (const auto& valy : delta_y_W_buffer){
        meanDelta_y_W += valy;
    }
    meanDelta_y_W/= delta_y_W_buffer.size();
    
    for (const auto& valz : delta_z_W_buffer){
        meanDelta_z_W += valz;
    }
    meanDelta_z_W/= delta_z_W_buffer.size();

    // force_disturbance.vector.x = meanDelta_x_W;
    // force_disturbance.vector.y = meanDelta_y_W;
    // force_disturbance.vector.z = meanDelta_z_W;

    const std::string filePath = "/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/log/disturbance_comparison.csv";
    std::ofstream save(filePath, std::ios::app);

    save<<std::setprecision(20)<<ros::Time::now().toSec()<<
        ","<<"ekf_dx_w"<<","<<force_disturbance.vector.x<<","<<
            "ekf_dy_w"<<","<<force_disturbance.vector.y<<","<<
            "ekf_dz_w"<<","<<force_disturbance.vector.z<<","<<
            "raw_dx_w"<<","<<delta_W.x()<<","<<
            "raw_dy_w"<<","<<delta_W.y()<<","<<
            "raw_dz_w"<<","<<delta_W.z()<<","<<
            "raw_dx_w_mean"<<","<<meanDelta_x_W<<","<<
            "raw_dy_w_mean"<<","<<meanDelta_y_W<<","<<
            "raw_dz_w_mean"<<","<<meanDelta_z_W<<","<<std::endl;
    save.close();

    if (cout_counter > 100){
    std::cout<<"=============== Raw disturbances ==============="<<std::endl;
    std::cout<<"Mean of delta_W_x: "<< meanDelta_x_W << " " 
    <<"Mean of delta_W_y: "<< meanDelta_y_W << " "
    <<"Mean of delta_W_z: "<< meanDelta_z_W << std::endl;

    std::cout<<"raw delta_W_x: "<< delta_W.x() << " " 
    <<"raw delta_W_y: "<< delta_W.y() << " "
    <<"raw delta_W_z: "<< delta_W.z() << std::endl;

    std::cout << "--------------------- System and Measurement states in EKF ------------------------" << std::endl;
    // std::cout << "disturbance_x_b: "<<system_states.disturbance_x<<" ms^-2 |disturbance_y_b: "<<system_states.disturbance_y<<" ms^-2 |disturbance_z: "<<system_states.disturbance_z<<" ms^-2"<<std::endl;
    std::cout << "disturbance_x_w: "<<Delta_W.x()<<" ms^-2 |disturbance_y_w: "<<Delta_W.y()<<" ms^-2 |disturbance_z: "<<Delta_W.z()<<" ms^-2"<<std::endl;
    // std::cout<<"acc_x: "<<accel.x_b<<" |acc_y: "<<accel.y_b<<" |acc_z: "<<accel.z_b<<std::endl;

    cout_counter = 0;
    }
    else{
        cout_counter++;
    }

    return force_disturbance;
}

// 4th order RK for integration
Eigen::MatrixXd DISTURBANCE_OBSERVER::RK4(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    Eigen::Matrix<double,6,1> k1;
    Eigen::Matrix<double,6,1> k2;
    Eigen::Matrix<double,6,1> k3;
    Eigen::Matrix<double,6,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
Eigen::MatrixXd DISTURBANCE_OBSERVER::f(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    // Define system dynamics
    Eigen::Matrix<double,6,1> xdot;    
    xdot << accel.x_b,accel.y_b,accel.z_b,                               // x_dot,y_dot,z_dot,u_dot,v_dot,w_dot
              // phi_dot,theta_dot,psi_dot
            0,0,0;                                                                // Disturbance_x, disturbance_y, disturbance_z in du, dv, dw
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
Eigen::MatrixXd DISTURBANCE_OBSERVER::h(Eigen::MatrixXd x)
{
    // Define measurement model
    Eigen::Matrix<double,6,1> y;
    y << x(0),x(1),x(2),  // u,v,w
        x(3),    // du, x(9) = disturbance_x in body frame 
        x(4),    // dv, x(10) = disturbance_y in body frame
        x(5)+(measurement_states.thrust_z/hover_thrust)*g;  // dw, x(11) = disturbance_z excluding gravity in body frame
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
Eigen::MatrixXd DISTURBANCE_OBSERVER::compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    // Define Jacobian of system dynamics
    Eigen::Matrix<double,6,6> F;
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
Eigen::MatrixXd DISTURBANCE_OBSERVER::compute_jacobian_H(Eigen::MatrixXd x)
{
    // Define Jacobian of measurement model
    Eigen::Matrix<double,6,6> H;
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
