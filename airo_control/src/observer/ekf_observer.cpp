#include "airo_control/observer/ekf_observer.h"

EKF::EKF(ros::NodeHandle& nh){
    nh.getParam("airo_control_node/ekf_observer/r_vel_x", R_VEL_X);
    nh.getParam("airo_control_node/ekf_observer/r_vel_y", R_VEL_Y);
    nh.getParam("airo_control_node/ekf_observer/r_vel_z", R_VEL_Z);
    nh.getParam("airo_control_node/ekf_observer/r_control_x", R_CONTROL_X);
    nh.getParam("airo_control_node/ekf_observer/r_control_y", R_CONTROL_Y);
    nh.getParam("airo_control_node/ekf_observer/r_control_z", R_CONTROL_Z);
    nh.getParam("airo_control_node/ekf_observer/q_vel_x", Q_VEL_X);
    nh.getParam("airo_control_node/ekf_observer/q_vel_y", Q_VEL_Y);
    nh.getParam("airo_control_node/ekf_observer/q_vel_z", Q_VEL_Z);
    nh.getParam("airo_control_node/ekf_observer/q_disturbance_x", Q_DISTURBANCE_X);
    nh.getParam("airo_control_node/ekf_observer/q_disturbance_y", Q_DISTURBANCE_Y);
    nh.getParam("airo_control_node/ekf_observer/q_disturbance_z", Q_DISTURBANCE_Z);
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

Eigen::Matrix3d EKF::q2ROT(const geometry_msgs::Quaternion q)
{
    Eigen::Quaterniond qd(
        q.w,
        q.x,
        q.y,
        q.z
    );

    return qd.toRotationMatrix();
}

geometry_msgs::AccelStamped EKF::observe(
    const geometry_msgs::PoseStamped& pose, 
    const geometry_msgs::TwistStamped& twist,
    const geometry_msgs::AccelStamped& imu, 
    const double& lala){
        
        // u,v,w in measurement and system states
        measurement_states.u = twist.twist.linear.x;   
        measurement_states.v = twist.twist.linear.y; 
        measurement_states.w = twist.twist.linear.z; 
        system_states.u = twist.twist.linear.x;   
        system_states.v = twist.twist.linear.y; 
        system_states.w = twist.twist.linear.z;
        
        geometry_msgs::AccelStamped accel_disturbance;

        // Linear acceleration 
        accel.x_b = imu.accel.linear.x;
        accel.y_b = imu.accel.linear.y;
        accel.z_b = imu.accel.linear.z;

        // Get U1 (thrust) per mass
        thrust_per_mass = lala;

        // Thrust from MPC output in in body frame
        //measurement_states.thrust_z = attitude_target.thrust;

        // Get input u
        input_u << accel.x_b, accel.y_b, accel.z_b;

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

        // Update u,v,w, and disturbances in system state
        system_states.u = esti_x(0);
        system_states.v = esti_x(1);
        system_states.w = esti_x(2);
        system_states.disturbance_x = esti_x(3);    // Unit: ms^-2
        system_states.disturbance_y = esti_x(4);
        system_states.disturbance_z = esti_x(5);

        Eigen::Vector3d Delta_B (system_states.disturbance_x,system_states.disturbance_y,system_states.disturbance_z); // Disturbance in body frame

        Eigen::Vector3d Delta_W = EKF::q2ROT(pose.pose.orientation)*Delta_B; // Disturbances in world frame
        
        // Safety range for disturbances
        if (Delta_W.x() > 4 || Delta_W.x() < -4) {
        Delta_W.x() = Delta_W.x() > 4 ? 4 : -4;
        }
        if (Delta_W.y() > 4 || Delta_W.y() < -4) {
        Delta_W.y() = Delta_W.y() > 4 ? 4 : -4;
        }
        if (Delta_W.z() > 4 || Delta_W.z() < -4) {
        Delta_W.z() = Delta_W.z() > 4 ? 4 : -4;
        }
        
        accel_disturbance.accel.linear.x = Delta_W.x();
        accel_disturbance.accel.linear.y = Delta_W.y();
        accel_disturbance.accel.linear.z = Delta_W.z();

        if (cout_counter > 100){
        std::cout << "--------------------- System and Measurement states in EKF ------------------------" << std::endl;
        std::cout << "disturbance_x_b: "<<system_states.disturbance_x<<" ms^-2 |disturbance_y_b: "<<system_states.disturbance_y<<" ms^-2 |disturbance_z: "<<system_states.disturbance_z<<" ms^-2"<<std::endl;
        std::cout << "disturbance_x_w: "<<Delta_W.x()<<" ms^-2 |disturbance_y_w: "<<Delta_W.y()<<" ms^-2 |disturbance_z: "<<Delta_W.z()<<" ms^-2"<<std::endl;
        std::cout<<"acc_x_b: "<<accel.x_b<<" |acc_y_b: "<<accel.y_b<<" |acc_z_b: "<<accel.z_b<<std::endl;

        cout_counter = 0;
        }
        else{
            cout_counter++;
        }

        return accel_disturbance;
    
}

// 4th order RK for integration
Eigen::MatrixXd EKF::RK4(Eigen::MatrixXd x, Eigen::MatrixXd u)
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
Eigen::MatrixXd EKF::f(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    // Define system dynamics
    Eigen::Matrix<double,6,1> xdot;    
    xdot << accel.x_b,accel.y_b,accel.z_b,              // u_dot,v_dot,w_dot
            0,0,0;                                      // Disturbance_x, disturbance_y, disturbance_z in du, dv, dw
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
Eigen::MatrixXd EKF::h(Eigen::MatrixXd x)
{
    // Define measurement model
    Eigen::Matrix<double,6,1> y;
    y << x(0),x(1),x(2),  // u,v,w
        x(3),    // du: disturbance_x in body frame 
        x(4),    // dv: disturbance_y in body frame
        x(5)+thrust_per_mass; // dw: disturbance_z excluding gravity in body frame
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
Eigen::MatrixXd EKF::compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u)
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
Eigen::MatrixXd EKF::compute_jacobian_H(Eigen::MatrixXd x)
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