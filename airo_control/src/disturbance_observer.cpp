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
    dt = 1/FSM_FREQUENCY;
 
    esti_x << 0,0,0,0,0,0,1e-6,1e-6,1e-6,0,0,0; // phi & theta CAN'T be 0
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
    const mavros_msgs::AttitudeTarget& u_B,
    const geometry_msgs::PoseStamped& pose
){
    Eigen::Vector3d delta_B;
    Eigen::Vector3d imu_B(
        imu_B_msg.accel.linear.x,
        imu_B_msg.accel.linear.y,
        imu_B_msg.accel.linear.z
    );

    Eigen::Matrix3d ROT_I2B = q2ROT(pose.pose.orientation).inverse();

    delta_B = imu_B - u_B.thrust / hover_thrust * Eigen::Vector3d(0.0,0.0,g);

    return delta_B;
}

geometry_msgs::Vector3Stamped DISTURBANCE_OBSERVER::observe(
    const geometry_msgs::PoseStamped& pose, 
    const geometry_msgs::TwistStamped& twist,
    const mavros_msgs::AttitudeTarget attitude_target, 
    const geometry_msgs::AccelStamped & imu
)
{
    std::cout<<"=============== here ==============="<<std::endl;
    Eigen::Vector3d delta_B = disturbance_raw(
        imu,
        attitude_target,
        pose
    );
    // std::cout<<
    //     "delta_B_x: "<<delta_B.x()<<' '
    //     <<"delta_B_y: "<<delta_B.y()<<' '
    //     <<"delta_B_z: "<<delta_B.z()<<' '<<std::endl<<std::endl;

    // std::cout<<"norm: "<<delta_B.norm()<<std::endl;

    Eigen::Vector3d delta_W = q2ROT(pose.pose.orientation) * delta_B;

    std::cout<<
        "delta_B_x: "<<delta_W.x()<<' '
        <<"delta_B_y: "<<delta_W.y()<<' '
        <<"delta_B_z: "<<delta_W.z()<<' '<<std::endl<<std::endl;

    std::cout<<"norm: "<<delta_W.norm()<<std::endl;


    geometry_msgs::Vector3Stamped lala;
    return lala;
}
// geometry_msgs::Vector3Stamped DISTURBANCE_OBSERVER::observe(const geometry_msgs::PoseStamped& pose, 
// const geometry_msgs::TwistStamped& twist,
// const mavros_msgs::AttitudeTarget attitude_target, 
// const geometry_msgs::AccelStamped & imu){
    
//     // x,y,z,u,v,w in measurement and system states
//     measurement_states.x = pose.pose.position.x; 
//     measurement_states.y = pose.pose.position.y;
//     measurement_states.z = pose.pose.position.z;
//     measurement_states.u = twist.twist.linear.x;   
//     measurement_states.v = twist.twist.linear.y; 
//     measurement_states.w = twist.twist.linear.z; 
   
//     system_states.x = pose.pose.position.x;
//     system_states.y = pose.pose.position.y;
//     system_states.z = pose.pose.position.z;
//     system_states.u = twist.twist.linear.x;   
//     system_states.v = twist.twist.linear.y; 
//     system_states.w = twist.twist.linear.z;

//     // p,q,r in system state
//     system_states.p = twist.twist.angular.x;
//     system_states.q = twist.twist.angular.y;
//     system_states.r = twist.twist.angular.z;
    
//     // phi,theta,psi in measurement and system states
//     Eigen::Vector3d current_euler = q2rpy(pose.pose.orientation);
//     // Rotation matrix
//     R_z << cos(current_euler.z()), sin(current_euler.z()), 0,
//         -sin(current_euler.z()), cos(current_euler.z()), 0,
//         0, 0, 1;
  
//     R_y << cos(current_euler.y()), 0, -sin(current_euler.y()),
//         0, 1, 0,
//         sin(current_euler.y()), 0, cos(current_euler.y());
  
//     R_x << 1, 0, 0,
//         0, cos(current_euler.x()), sin(current_euler.x()),
//         0, -sin(current_euler.x()), cos(current_euler.x());  

//     R_b2w = R_z * R_y * R_x;

//     euler_body << current_euler.x(),current_euler.y(), current_euler.z();
//     euler_world = R_b2w*euler_body;

//     measurement_states.phi = euler_world(0,0);
//     measurement_states.theta = euler_world(1,0);
//     measurement_states.psi = euler_world(2,0);

//     system_states.phi = euler_world(0,0);
//     system_states.theta = euler_world(1,0);
//     system_states.psi = euler_world(2,0);

//     // phi_dot,theta_dot,psi_dot in system states
//     system_states.phi_dot_b = system_states.p  
//                         + (sin(current_euler.x())*tan(current_euler.y()))*system_states.q
//                         + (cos(current_euler.x())*tan(current_euler.y()))*system_states.r;

//     system_states.theta_dot_b = system_states.q*cos(current_euler.x())-system_states.r*sin(current_euler.x());

//     system_states.psi_dot_b = (sin(current_euler.x())/cos(current_euler.y()))*system_states.q
//                             + (cos(current_euler.x())/cos(current_euler.y()))*system_states.r;

//     euler_dot_body << system_states.phi_dot_b, system_states.theta_dot_b, system_states.psi_dot_b;
//     euler_dot_world = R_b2w*euler_dot_body;

//     system_states.phi_dot_w = euler_dot_world(0,0);
//     system_states.theta_dot_w = euler_dot_world(1,0);
//     system_states.psi_dot_w = euler_dot_world(2,0);

//     // disturbances in system state
//     geometry_msgs::Vector3Stamped force_disturbance;
//     force_disturbance.vector.x = system_states.disturbance_x;
//     force_disturbance.vector.y = system_states.disturbance_y;
//     force_disturbance.vector.z = system_states.disturbance_z;

//     // U1(thrust) in measurement state
//     measurement_states.thrust_x = attitude_target.thrust;
//     measurement_states.thrust_y = attitude_target.thrust;
//     measurement_states.thrust_z = attitude_target.thrust;

    
    
//     // Linear acceleration 
//     accel_body << imu.accel.linear.x, imu.accel.linear.y, imu.accel.linear.z-g;
//     accel_world = R_b2w * accel_body;

//     accel.x = accel_world(0,0);
//     accel.y = accel_world(1,0);
//     accel.z = accel_world(2,0);

//     // U1(thrust) in measurement state
//     thrust_body << attitude_target.thrust, attitude_target.thrust, attitude_target.thrust;
//     thrust_world = R_b2w * thrust_body;

//     // Thrust from MPC output in in world frame
//     measurement_states.thrust_x = thrust_world(0,0)*(cos(current_euler.x())*sin(current_euler.y()*cos(current_euler.z())+sin(current_euler.x())*sin(current_euler.z())));
//     measurement_states.thrust_y = thrust_world(1,0)*(cos(current_euler.x())*sin(current_euler.y()*sin(current_euler.z())-sin(current_euler.x())*cos(current_euler.z())));
//     measurement_states.thrust_z = thrust_world(2,0)*(cos(current_euler.x())*cos(current_euler.y()));

//     // Thrust from MPC output in in body frame
//     // measurement_states.thrust_x = thrust_body(0,0); 
//     // measurement_states.thrust_y = thrust_body(1,0);
//     // measurement_states.thrust_z = thrust_body(2,0);

//     // Get input u and measurment y
//     input_u << measurement_states.thrust_x, measurement_states.thrust_y, measurement_states.thrust_z; 
   
//     meas_y << measurement_states.x, measurement_states.y, measurement_states.z,
//                 measurement_states.u, measurement_states.v, measurement_states.w,
//                 measurement_states.phi, measurement_states.theta, measurement_states.psi,
//                 input_u[0], input_u[1], input_u[2];

//     // Define Jacobian matrices of system dynamics and measurement model
//     Eigen::Matrix<double,12,12> F;     // Jacobian of system dynamics
//     Eigen::Matrix<double,12,12> H;     // Jacobian of measurement model

//     // Define Kalman gain matrix
//     Eigen::Matrix<double,12,12> Kal;

//     // Define prediction and update steps
//     Eigen::Matrix<double,12,1> x_pred;     // predicted state
//     Eigen::Matrix<double,12,12> P_pred;    // predicted covariance
//     Eigen::Matrix<double,12,1> y_pred;     // predicted measurement
//     Eigen::Matrix<double,12,1> y_err;      // measurement error

//     // Prediction step: estimate state and covariance at time k+1|k
//     F = compute_jacobian_F(esti_x, input_u);             // compute Jacobian of system dynamics at current state and input
//     x_pred = RK4(esti_x, input_u);                       // predict state at time k+1|k
//     P_pred = F * esti_P * F.transpose() + Q_noise;       // predict covariance at time k+1|k

    
//     // Update step: correct state and covariance using measurement at time k+1
//     H = compute_jacobian_H(x_pred);                     // compute Jacobian of measurement model at predicted state    
//     y_pred = h(x_pred);                                 // predict measurement at time k+1
//     y_err = meas_y - y_pred;                            // compute measurement error
//     Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + R_noise).inverse();    // compute Kalman gain
//     esti_x = x_pred + Kal * y_err;                      // correct state estimate
//     esti_P = (Eigen::MatrixXd::Identity(m, m) - Kal * H) * P_pred * (Eigen::MatrixXd::Identity(m, m) - Kal * H).transpose() + Kal*R_noise*Kal.transpose(); // correct covariance estimate

//     // Update disturbance_x in system state
//     system_states.x = esti_x(0);
//     system_states.y = esti_x(1);
//     system_states.z = esti_x(2);
//     system_states.u = esti_x(3);
//     system_states.v = esti_x(4);
//     system_states.w = esti_x(5);
//     system_states.phi = esti_x(6);
//     system_states.theta = esti_x(7);
//     system_states.psi = esti_x(8);
//     force_disturbance.vector.x = esti_x(9);    // Unit: ms^-2
//     force_disturbance.vector.y = esti_x(10);
//     force_disturbance.vector.z = esti_x(11);

//     // //Update previous u,v,w
//     // pre_linear_v[0] = twist.twist.linear.x;
//     // pre_linear_v[1] = twist.twist.linear.y;
//     // pre_linear_v[2] = twist.twist.linear.z;

//     // std::ofstream save("/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/tracking.csv", std::ios::app);
//     // save<<std::setprecision(20)<<ros::Time::now().toSec()<<
//     //     ","<<"x"<<","<<system_states.x<<","<<measurement_states.x<<","<<
//     //         "y"<<","<<system_states.y<<","<<measurement_states.y<<","<<
//     //         "z"<<","<<system_states.z<<","<<measurement_states.z<<","<<
//     //         "u"<<","<<system_states.u<<","<<measurement_states.u<<","<<
//     //         "v"<<","<<system_states.v<<","<<measurement_states.v<<","<<
//     //         "w"<<","<<system_states.w<<","<<measurement_states.w<<","<<
//     //         "phi"<<","<<system_states.phi<<","<<measurement_states.phi<<","<<
//     //         "theta"<<","<<system_states.theta<<","<<measurement_states.theta<<","<<
//     //         "psi"<<","<<system_states.psi<<","<<measurement_states.psi<<","<<std::endl;
//     // save.close();

//     std::ofstream save("/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/log/disturbance_tracking.csv", std::ios::app);
//     save<<std::setprecision(20)<<ros::Time::now().toSec()<<
//         ","<<"disturbance_x"<<","<<force_disturbance.vector.x<<","<<
//             "disturbance_y"<<","<<force_disturbance.vector.y<<","<<
//             "disturbance_z"<<","<<force_disturbance.vector.z<<","<<std::endl;
//     save.close();

//     if (cout_counter > 100){
//         std::cout << "--------------------- System and Measurement states in EKF ------------------------" << std::endl;
//     // std::cout << "state_x: "<<system_states.x<< " state_y: "<<system_states.y<<" state_z: "<<system_states.z<<std::endl;
//     // std::cout << "meas_x: "<<measurement_states.x<< " meas_y: "<<measurement_states.y<<" meas_z: "<<measurement_states.z<<std::endl;
//     // std::cout << "state_u: "<<system_states.u<< " state_v: "<<system_states.v<<" state_w: "<<system_states.w<<std::endl;
//     // std::cout << "meas_u: "<<measurement_states.u<< " meas_v: "<<measurement_states.v<<" meas_w: "<<measurement_states.w<<std::endl;
//     // std::cout << "state_phi: "<<system_states.phi<< " state_theta: "<<system_states.theta<<" state_psi: "<<system_states.psi<<std::endl;
//     // std::cout << "meas_phi: "<<measurement_states.phi<< " meas_theta: "<<measurement_states.theta<<" meas_psi: "<<measurement_states.psi<<std::endl;
//     std::cout << "disturbance_x: "<<force_disturbance.vector.x<<" ms^-2 |disturbance_y: "<<force_disturbance.vector.y<<" ms^-2 |disturbance_z: "<<force_disturbance.vector.z<<" ms^-2"<<std::endl;
//     std::cout << "disturbance_x: "<<force_disturbance.vector.x*mass<<" N |disturbance_y: "<<force_disturbance.vector.y*mass<<" N |disturbance_z: "<<force_disturbance.vector.z*mass<<" N"<<std::endl;
//     std::cout << "U1_x: "<<measurement_states.thrust_x<<" |U1_y: "<<measurement_states.thrust_y<<" |U1_z: "<<measurement_states.thrust_z<<std::endl;
//     std::cout<< "U1: "<<attitude_target.thrust<<std::endl;
//     std::cout<<"acc_x: "<<accel.x<<" |acc_y: "<<accel.y<<" |acc_z: "<<accel.z<<std::endl;
//     std::cout<<"acc_x_imu: "<<imu.accel.linear.x<<" |acc_y_imu: "<<imu.accel.linear.y<<" |acc_z_imu: "<<imu.accel.linear.z<<std::endl;

//     // std::cout<<"mass = hover_thrust/g = "<<hover_thrust/g<<std::endl;

//     cout_counter = 0;
//     }
//     else{
//         cout_counter++;
//     }

//     return force_disturbance;
// }

// 4th order RK for integration
Eigen::MatrixXd DISTURBANCE_OBSERVER::RK4(Eigen::MatrixXd x, Eigen::MatrixXd u)
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
Eigen::MatrixXd DISTURBANCE_OBSERVER::f(Eigen::MatrixXd x, Eigen::MatrixXd u)
{
    // Define system dynamics
    Eigen::Matrix<double,12,1> xdot;    
    xdot << x(3),x(4),x(5),accel.x,accel.y,accel.z,                               // x_dot,y_dot,z_dot,u_dot,v_dot,w_dot
            system_states.phi_dot_w,system_states.theta_dot_w,system_states.psi_dot_w,  // phi_dot,theta_dot,psi_dot
            0,0,0;                                                                // Disturbance_x, disturbance_y, disturbance_z in du, dv, dw
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
Eigen::MatrixXd DISTURBANCE_OBSERVER::h(Eigen::MatrixXd x)
{
    // Define measurement model
    Eigen::Matrix<double,12,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),  // x,y,z,u,v,w,phi,theta,psi
        // (accel.x-x(9))*(hover_thrust)/((g)*(cos(x(6))*sin(x(7)*cos(x(8))+sin(x(6))*sin(x(8))))),   // thrust for du, x(9) = disturbance_x in body frame 
        // (accel.y-x(10))*(hover_thrust)/((g)*(cos(x(6))*sin(x(7)*sin(x(8))-sin(x(6))*cos(x(8))))),  // thrust for dv, x(10) = disturbance_y in body frame
        // (accel.z-x(11)+g)*(hover_thrust)/((g)*(cos(x(6))*cos(x(7))));                              // thrust for dw, x(11) = disturbance_z excluding gravity in body frame
        (accel.x-x(9))*(hover_thrust)/((g)),     // thrust for du, x(9) = disturbance_x in world frame 
        (accel.y-x(10))*(hover_thrust)/((g)),    // thrust for dv, x(10) = disturbance_y in world frame
        (accel.z-x(11)+g)*(hover_thrust)/((g));  // thrust for dw, x(11) = disturbance_z excluding gravity in world frame
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
Eigen::MatrixXd DISTURBANCE_OBSERVER::compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u)
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
Eigen::MatrixXd DISTURBANCE_OBSERVER::compute_jacobian_H(Eigen::MatrixXd x)
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
