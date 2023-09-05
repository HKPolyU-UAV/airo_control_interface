#include "airo_control/controller/backstepping.h"

BACKSTEPPING::BACKSTEPPING(ros::NodeHandle& nh){
    // Get Parameters
    nh.getParam("airo_control_node/backstepping/hover_thrust",param.hover_thrust);
    nh.getParam("airo_control_node/backstepping/show_debug",param.show_debug);
    nh.getParam("airo_control_node/backstepping/k_x1",param.k_x1);
    nh.getParam("airo_control_node/backstepping/k_x2",param.k_x2);
    nh.getParam("airo_control_node/backstepping/k_y1",param.k_y1);
    nh.getParam("airo_control_node/backstepping/k_y2",param.k_y2);
    nh.getParam("airo_control_node/backstepping/k_z1",param.k_z1);
    nh.getParam("airo_control_node/backstepping/k_z2",param.k_z2);
} 

void BACKSTEPPING::print(){
    // std::cout << "------------------------------------------------------------------------------" << std::endl;
    // std::cout << "x_ref:      " << acados_in.yref[0][0] << "\ty_ref:      " << acados_in.yref[0][1] << "\tz_ref:         " << acados_in.yref[0][2] << std::endl;
    // std::cout << "x_gt:       " << acados_in.x0[x] << "\ty_gt:       " << acados_in.x0[y] << "\tz_gt:          " << acados_in.x0[z] << std::endl;
    // std::cout << "theta_cmd:  " << target_euler.y() << "\tphi_cmd:    " << target_euler.x() <<  "\tpsi_cmd:       " << target_euler.z() << std::endl;
    // std::cout << "theta_gt:   " << current_euler.y() << "\tphi_gt:     " << current_euler.x() <<  "\tpsi_gt:        " << current_euler.z() << std::endl;
    // std::cout << "thrust_cmd: " << attitude_target.thrust << "\tsolve_time: "<< acados_out.cpu_time  << "\tacados_status: " << acados_out.status << std::endl;
    // std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
    // std::cout << "------------------------------------------------------------------------------" << std::endl;
}

void BACKSTEPPING::show_debug(){
    if (param.show_debug){
        if(debug_counter > 2){ //reduce cout rate
            BACKSTEPPING::print();
            debug_counter = 0;
        }
        else{
            debug_counter++;
        }
    }
}

mavros_msgs::AttitudeTarget BACKSTEPPING::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::Reference& ref){  
    current_euler = q2rpy(current_pose.pose.orientation);
    ref_euler = q2rpy(ref.ref_pose.orientation);

    // Altitude Control   
    e_z1 = ref.ref_pose.position.z - current_pose.pose.position.z;
    e_z2 = current_twist.twist.linear.z - ref.ref_twist.linear.z - param.k_z1*e_z1;
    attitude_target.thrust = param.hover_thrust/(g*cos(current_euler.x())*cos(current_euler.y())) * (e_z1+g+ref.ref_accel.linear.z-param.k_z1*(e_z2 + param.k_z1*e_z1)-param.k_z2*e_z2);

    // X Translation Control
    e_x1 = ref.ref_pose.position.x - current_pose.pose.position.x;
    e_x2 = current_twist.twist.linear.x - ref.ref_twist.linear.x - param.k_x1*e_x1;
    u_x = param.hover_thrust/(attitude_target.thrust*g)*(e_x1+ref.ref_accel.linear.x-param.k_x1*(e_x2+param.k_x1*e_x1)-param.k_x2*e_x2);

    // Y Translation Control
    e_y1 = ref.ref_pose.position.y - current_pose.pose.position.y;
    e_y2 = current_twist.twist.linear.y - ref.ref_twist.linear.y - param.k_y1*e_y1;
    u_y = param.hover_thrust/(attitude_target.thrust*g)*(e_y1+ref.ref_accel.linear.y-param.k_y1*(e_y2+param.k_y1*e_y1)-param.k_y2*e_y2);

    // Calculate Targer Eulers
    target_euler.x() = asin(u_x*sin(ref_euler.z()) - u_y*cos(ref_euler.z()));
    target_euler.y() = asin((u_x*cos(ref_euler.z()) + u_y*sin(ref_euler.z()))/cos(target_euler.x()));
    target_euler.z() = ref_euler.z();
    attitude_target.orientation = rpy2q(target_euler);

    show_debug();
    return attitude_target;
}