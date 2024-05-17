#include "airo_control/controller/backstepping.h"

BACKSTEPPING::BACKSTEPPING(ros::NodeHandle& nh){
    // Get Parameters
    nh.getParam("airo_control_node/backstepping/pub_debug",param.pub_debug);
    nh.getParam("airo_control_node/backstepping/enable_thrust_model",param.enable_thrust_model);
    nh.getParam("airo_control_node/backstepping/hover_thrust",param.hover_thrust);
    nh.getParam("airo_control_node/backstepping/k_x1",param.k_x1);
    nh.getParam("airo_control_node/backstepping/k_x2",param.k_x2);
    nh.getParam("airo_control_node/backstepping/k_y1",param.k_y1);  
    nh.getParam("airo_control_node/backstepping/k_y2",param.k_y2);
    nh.getParam("airo_control_node/backstepping/k_z1",param.k_z1);
    nh.getParam("airo_control_node/backstepping/k_z2",param.k_z2);
    
    if (param.enable_thrust_model){
        nh.getParam("airo_control_node/thrust_model/mass",thrust_model.mass);
        nh.getParam("airo_control_node/thrust_model/K1",thrust_model.K1);
        nh.getParam("airo_control_node/thrust_model/K2",thrust_model.K2);
        nh.getParam("airo_control_node/thrust_model/K3",thrust_model.K3);
    }

    debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/airo_control/backstepping/debug",1);
} 

void BACKSTEPPING::pub_debug(){
    std_msgs::Float64MultiArray debug_msg;
    debug_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    debug_msg.layout.dim[0].size = 8;
    debug_msg.layout.dim[0].stride = 1;
    debug_msg.data.clear();
    debug_msg.data.push_back(e_z1);
    debug_msg.data.push_back(e_z2);
    debug_msg.data.push_back(e_x1);
    debug_msg.data.push_back(e_x2);
    debug_msg.data.push_back(u_x);
    debug_msg.data.push_back(e_y1);
    debug_msg.data.push_back(e_y2);
    debug_msg.data.push_back(u_y);

    debug_pub.publish(debug_msg);  
}

mavros_msgs::AttitudeTarget BACKSTEPPING::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::ReferenceStamped& ref, const sensor_msgs::BatteryState& battery_state){  
    current_euler = q2rpy(current_pose.pose.orientation);
    ref_euler = q2rpy(ref.ref.pose.orientation);

    // Altitude Control
    e_z1 = ref.ref.pose.position.z - current_pose.pose.position.z;
    e_z2 = current_twist.twist.linear.z - ref.ref.twist.linear.z - param.k_z1*e_z1;
    a_z = (e_z1+g+ref.ref.accel.linear.z-param.k_z1*(e_z2 + param.k_z1*e_z1)-param.k_z2*e_z2)/(cos(current_euler.x())*cos(current_euler.y()));
    attitude_target.thrust = inverse_thrust_model(a_z,battery_state.voltage,param,thrust_model);

    // X Translation Control
    e_x1 = ref.ref.pose.position.x - current_pose.pose.position.x;
    e_x2 = current_twist.twist.linear.x - ref.ref.twist.linear.x - param.k_x1*e_x1;
    u_x = (e_x1+ref.ref.accel.linear.x-param.k_x1*(e_x2+param.k_x1*e_x1)-param.k_x2*e_x2)/a_z;

    // Y Translation Control
    e_y1 = ref.ref.pose.position.y - current_pose.pose.position.y;
    e_y2 = current_twist.twist.linear.y - ref.ref.twist.linear.y - param.k_y1*e_y1;
    u_y = (e_y1+ref.ref.accel.linear.y-param.k_y1*(e_y2+param.k_y1*e_y1)-param.k_y2*e_y2)/a_z;

    // Calculate Targer Eulers
    target_euler.x() = asin(u_x*sin(ref_euler.z()) - u_y*cos(ref_euler.z()));
    target_euler.y() = asin((u_x*cos(ref_euler.z()) + u_y*sin(ref_euler.z()))/cos(target_euler.x()));
    target_euler.z() = ref_euler.z();
    attitude_target.orientation = rpy2q(target_euler);

    if (param.pub_debug){
        pub_debug();
    }

    return attitude_target;
}

double BACKSTEPPING::get_hover_thrust(){
    return param.hover_thrust;
}