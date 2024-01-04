#include "airo_control/controller/observer_ekf.h"

OBSERVER_EKF::OBSERVER_EKF(ros::NodeHandle& nh){
    // Get EKF parameters
    nh.getParam("airo_control_node/observer_ekf/hover_thrust", param.hover_thrust);
    nh.getParam("airo_control_node/observer_ekf/tau_phi", param.tau_phi);
    nh.getParam("airo_control_node/observer_ekf/tau_theta", param.tau_theta);
    nh.getParam("airo_control_node/observer_ekf/tau_psi", param.tau_psi);
    nh.getParam("airo_control_node/observer_ekf/disturbance_x", solverparam.disturbance_x);
    nh.getParam("airo_control_node/observer_ekf/disturbance_y", solverparam.disturbance_y);
    nh.getParam("airo_control_node/observer_ekf/disturbance_z", solverparam.disturbance_z);

    Q_cov << pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),
            pow(dt,2),pow(dt,2),pow(dt,2);
    noise_Q= Q_cov.asDiagonal();

    esti_x << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    esti_P = P0;

    // ROS Sub & Pub
    esti_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/airco_control/ekf/pose",5);
    esti_disturbance_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_control/ekf/disturbance",5);
    applied_disturbance_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_control/applied_disturbance",5);

}

void OBSERVER_EKF::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    // local_pose.header.stamp = msg->header.stamp;
    // local_pose.pose = msg->pose;

    // Get linear position x,y,z
    local_pos.x = pose->pose.position.x;
    local_pos.y = pose->pose.position.y;
    local_pos.z = pose->pose.position.z;

    // Get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);
}

void OBSERVER_EKF::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& pose){
    
    // Get linear velocity u,v,w
    local_vel.u = pose->twist.linear.x;
    local_vel.v = pose->twist.linear.y;
    local_vel.w = pose->twist.linear.z;

    // Get angular velocity p,q
    local_vel.p = pose->twist.angular.x;
    local_vel.q = pose->twist.angular.y;

    // Inertial frame velocity to body frame
    Matrix<double,3,1> v_linear_inertial;

    v_linear_body << local_vel.u, local_vel.v, local_vel.w;
    v_angular_body << local_vel.p, local_vel.q, local_vel.r;

    // R_ib << 0,1,0,
    //         1,0,0,
    //         0,0,-1;

    // v_linear_body = R_ib.inverse()*v_linear_inertial;

    body_acc.x = (v_linear_body[0]-pre_body_vel.u)/dt;  // du
    body_acc.y = (v_linear_body[1]-pre_body_vel.v)/dt;  // dv
    body_acc.z = (v_linear_body[2]-pre_body_vel.w)/dt;  // dw

    pre_body_vel.u = v_linear_body[0];
    pre_body_vel.v = v_linear_body[1];
    pre_body_vel.w = v_linear_body[2];
    pre_body_vel.p = v_angular_body[0];
    pre_body_vel.q = v_angular_body[1];
    // pre_body_vel.r = v_angular_body[3];

    // Matrix<double,3,1> compensate_f_inertial;
    // Matrix<double,3,1> compensate_f_body;
    // compensate_f_inertial << 20,0,0;
    // compensate_f_body = R_ib.inverse()*compensate_f_inertial;

}

// Quaternion to Euler angle
OBSERVER_EKF::Euler OBSERVER_EKF::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion, tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// Euler angle to Quaternion
geometry_msgs::Quaternion OBSERVER_EKF::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

void OBSERVER_EKF::solve(){

}

void OBSERVER_EKF::EKF(){
    // Get input u and measurement y
    meas_u << current_thrust.thrust0, current_thrust.thrust1, current_thrust.thrust2; // thrust for du, dv, dw
    
    // Measured state [x, y, z, u, v, w, phi, theta, psi, dphi, dtheta, disturbance_x, disturbance_y, disturbance_z]
    meas_y << local_pos.x, local_pos.y, local_pos.z, local_vel.u, local_vel.v, local_vel.w,
              local_euler.phi, local_euler.theta, local_euler.psi, local_vel.p, local_vel.q,
              solverparam.disturbance_x, solverparam.disturbance_y, solverparam.disturbance_z;
             

    // Define Jacobian matrices of system dynamics and measurement model
    Matrix<double,14,14> F;                             // Jacobian of system dynamics
    Matrix<double,14,14> H;                             // Jacobian of measurement model

    // Define Kalman gain matrix
    Matrix<double,14,14> Kal;

    // Define prediction and update steps
    Matrix<double,14,1> x_pred;                         // Predicted state
    Matrix<double,14,14> P_pred;                        // Predicted covariance
    Matrix<double,14,1> y_pred;                         // Predicted measurement
    Matrix<double,14,1> y_err;                          // Measurement error

    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, meas_u);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, meas_u);                       // predict state at time k+1|k
    // dx = f(esti_x, meas_u);                             // acceleration
    P_pred = F * esti_P * F.transpose() + noise_Q;      // predict covariance at time k+1|k

    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                     // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred);                                 // predict measurement at time k+1
    y_err = meas_y - y_pred;                            // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                      // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred * (MatrixXd::Identity(n, n) - Kal * H).transpose() + Kal*noise_R*Kal.transpose(); // correct covariance estimate

    // body frame disturbance to inertial frame
    // wf_disturbance << (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
    //         (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13) + (-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
    //         (-sin(meas_y(4)))*esti_x(12) + (cos(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
    //         esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16) + cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
    //         (cos(meas_y(3)))*esti_x(16) + (sin(meas_y(3)))*esti_x(17),
    //         (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16) + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);
    
    // publish estimate pose
    tf2::Quaternion quat;
    quat.setRPY(esti_x(6), esti_x(7), esti_x(8));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    esti_pose.pose.position.x = esti_x(0);      // x
    esti_pose.pose.position.y = esti_x(1);      // y
    esti_pose.pose.position.z = esti_x(2);      // z
    esti_twist.twist.linear.x = esti_x(3);      // u
    esti_twist.twist.linear.y = esti_x(4);      // v
    esti_twist.twist.linear.z = esti_x(5);      // w
    esti_pose.pose.orientation.x = quat_msg.x;  
    esti_pose.pose.orientation.y = quat_msg.y;
    esti_pose.pose.orientation.z = quat_msg.z;
    esti_pose.pose.orientation.w = quat_msg.w;
    esti_twist.twist.angular.x = esti_x(9);     // p
    esti_twist.twist.angular.y = esti_x(10);    // q
    // esti_twist.twist.angular.z = esti_x(11);    // r
    esti_pose.header.stamp = ros::Time::now();
    // esti_pose.header.frame_id = "odom_frame";
    // esti_pose.child_frame_id = "base_link";
    esti_pose_pub.publish(esti_pose);

    // Publish estimate disturbance
    esti_disturbance.pose.position.x = wf_disturbance(0);
    esti_disturbance.pose.position.y = wf_disturbance(1);
    esti_disturbance.pose.position.z = wf_disturbance(2);
    esti_disturbance.header.stamp = ros::Time::now();
    // // esti_disturbance.header.frame_id = "odom_frame";
    // // esti_disturbance.child_frame_id = "base_link";
    esti_disturbance_pub.publish(esti_disturbance);

    // // publish estimate disturbance
    // applied_disturbance.pose.position.x = applied_wrench.fx;
    // applied_disturbance.pose.position.y = applied_wrench.fy;
    // applied_disturbance.pose.position.z = applied_wrench.fz;
    // applied_disturbance.header.stamp = ros::Time::now();
    // // applied_disturbance.header.frame_id = "odom_frame";
    // // applied_disturbance.child_frame_id = "base_link";
    // applied_disturbance_pub.publish(applied_disturbance);

    // // print estimate disturbance
    // if(cout_counter > 2){
    //     std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
    //     // std::cout << "esti_x12:   " << esti_x(12) << "\t esti_x2:  " << esti_x(2) << std::endl;
    //     //std::cout << "tau_x:  " << meas_y(12) << "  tau_y:  " << meas_y(13) << "  tau_z:  " << meas_y(14) << "  tau_psi:  " << meas_y(17) << std::endl;
    //     std::cout << "acc_x:  " << body_acc.x << "  acc_y:  " << body_acc.y << "  acc_z:  " << body_acc.z << std::endl;
    //     // std::cout << "acc_phi:  " << body_acc.phi << "  acc_theta:  " << body_acc.theta << "  acc_psi:  " << body_acc.psi << std::endl;
    //     //std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_z:    " << acados_in.yref[0][2] << "\tref_yaw:    " << yaw_ref << std::endl;
    //     std::cout << "pos_x: " << meas_y(0) << "  pos_y: " << meas_y(1) << "  pos_z: " << meas_y(2) << " phi: " << meas_y(3) << "  theta: " << meas_y(4) << "  psi: " << meas_y(5) <<std::endl;
    //     std::cout << "esti_x: " << esti_x(0) << "  esti_y: " << esti_x(1) << "  esti_z: " << esti_x(2) << " esti_phi: " << esti_x(3) << "  esti_theta: " << esti_x(4) << "  esti_psi: " << esti_x(5) <<std::endl;
    //     //std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_z:  " << error_pose.pose.pose.position.z << std::endl;
    //     // std::cout << "applied force x:  " << applied_wrench.fx << "\tforce y:  " << applied_wrench.fy << "\tforce_z:  " << applied_wrench.fz << std::endl;
    //     //std::cout << "applied torque x:  " << applied_wrench.tx << "\ttorque y:  " << applied_wrench.ty << "\ttorque_z:  " << applied_wrench.tz << std::endl;
    //     //std::cout << "(body frame) disturbance x: " << esti_x(12) << "    disturbance y: " << esti_x(13) << "    disturbance z: " << esti_x(14) << std::endl;
    //     //std::cout << "(world frame) disturbance x: " << wf_disturbance(0) << "    disturbance y: " << wf_disturbance(1) << "    disturbance z: " << wf_disturbance(2) << std::endl;
    //     //std::cout << "(world frame) disturbance phi: " << wf_disturbance(3) << "    disturbance theta: " << wf_disturbance(4) << "    disturbance psi: " << wf_disturbance(5) << std::endl;
    //     //std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
    //     //std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
    //     std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
    //     cout_counter = 0;
    // }
    // else{
    //     cout_counter++;
    // }
}

// 4th order RK for integration
MatrixXd OBSERVER_EKF::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,14,1> k1;
    Matrix<double,14,1> k2;
    Matrix<double,14,1> k3;
    Matrix<double,14,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
MatrixXd OBSERVER_EKF::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,14,1> xdot;

    geometry_msgs::Quaternion target_quaternion = BASE_CONTROLLER::rpy2q(target_euler);
    
    // KAu = K*u;
    xdot << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8),                                 // x,y,z,u,v,w,phi,theta,psi
            (target_euler.x() - x(6)) / param.tau_phi,                                            // p = dphi
            (target_euler.y() - x(7)) / param.tau_theta,                                          // q = dtheta
            // (cos(x(6))*sin(x(7))*cos(x(8)) + sin(x(6))*sin(x(8))) * attitude_target.thrust/param.hover_thrust*g+solverparam.disturbance_x,   // du
            // (cos(x(6))*sin(x(7))*sin(x(8)) - sin(x(6))*cos(x(8))) * attitude_target.thrust/param.hover_thrust*g+solverparam.disturbance_y,   // dv
            // -g + cos(x(7)) * cos(x(6)) * attitude_target.thrust/param.hover_thrust*g+solverparam.disturbance_z,                              // dw
            0,0,0;                                                                                // Disturbance_x, disturbance_y, disturbance_z in du, dv, dw
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd OBSERVER_EKF::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,14,1> y;
    y << x(0), x(1), x(2), x(3),x(4),x(5),  // x,y,z,u,v,w
        x(6),x(7),x(8),x(9),x(10), // phi,theta,psi
        (body_acc.x-x(11))*(param.hover_thrust)/((g)*(cos(x(6))*sin(x(7)*cos(x(8))+sin(x(6))*sin(x(8))))),   // thrust for du     
        (body_acc.y-x(12))*(param.hover_thrust)/((g)*(cos(x(6))*sin(x(7))*sin(x(8))-sin(x(6))*cos(x(8)))),   // thrust for dv
        (body_acc.z-x(13)+g)*(param.hover_thrust)/((g)*(cos(x(6))*cos(x(7))));                               // thrust for dw
    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd OBSERVER_EKF::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,14,14> F;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd OBSERVER_EKF::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,14,14> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}