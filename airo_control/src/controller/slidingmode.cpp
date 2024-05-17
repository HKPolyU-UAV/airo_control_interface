#include "airo_control/controller/slidingmode.h"

SLIDINGMODE::SLIDINGMODE(ros::NodeHandle& nh){
    // Get Parameters
    nh.getParam("airo_control_node/slidingmode/pub_debug",param.pub_debug);
    nh.getParam("airo_control_node/slidingmode/enable_thrust_model",param.enable_thrust_model);
    nh.getParam("airo_control_node/slidingmode/hover_thrust",param.hover_thrust);
    nh.getParam("airo_control_node/slidingmode/k_xe",param.k_xe);
    nh.getParam("airo_control_node/slidingmode/k_xs",param.k_xs);
    nh.getParam("airo_control_node/slidingmode/k_xt",param.k_xt);
    nh.getParam("airo_control_node/slidingmode/k_ye",param.k_ye);
    nh.getParam("airo_control_node/slidingmode/k_ys",param.k_ys);
    nh.getParam("airo_control_node/slidingmode/k_yt",param.k_yt);
    nh.getParam("airo_control_node/slidingmode/k_ze",param.k_ze);
    nh.getParam("airo_control_node/slidingmode/k_zs",param.k_zs);
    nh.getParam("airo_control_node/slidingmode/k_zt",param.k_zt);

    if (param.enable_thrust_model){
        nh.getParam("airo_control_node/thrust_model/mass",thrust_model.mass);
        nh.getParam("airo_control_node/thrust_model/K1",thrust_model.K1);
        nh.getParam("airo_control_node/thrust_model/K2",thrust_model.K2);
        nh.getParam("airo_control_node/thrust_model/K3",thrust_model.K3);
    }

    debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/airo_control/slidingmode/debug",1);
}

void SLIDINGMODE::pub_debug(){
    std_msgs::Float64MultiArray debug_msg;
    debug_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    debug_msg.layout.dim[0].size = 11;
    debug_msg.layout.dim[0].stride = 1;
    debug_msg.data.clear();
    debug_msg.data.push_back(e_z);
    debug_msg.data.push_back(e_dz);
    debug_msg.data.push_back(s_z);
    debug_msg.data.push_back(e_x);
    debug_msg.data.push_back(e_dx);
    debug_msg.data.push_back(s_x);
    debug_msg.data.push_back(u_x);
    debug_msg.data.push_back(e_y);
    debug_msg.data.push_back(e_dy);
    debug_msg.data.push_back(s_y);
    debug_msg.data.push_back(u_y);

    debug_pub.publish(debug_msg);
}

mavros_msgs::AttitudeTarget SLIDINGMODE::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::ReferenceStamped& ref, const sensor_msgs::BatteryState& battery_state){  
    current_euler = q2rpy(current_pose.pose.orientation);
    ref_euler = q2rpy(ref.ref.pose.orientation);

    // Altitude Control
    e_z = ref.ref.pose.position.z - current_pose.pose.position.z;
    e_dz= ref.ref.twist.linear.z - current_twist.twist.linear.z;
    s_z = param.k_ze * e_z + e_dz;
    a_z = (param.k_ze*e_dz+ref.ref.accel.linear.z+g+param.k_zs*tanh(param.k_zt*s_z))/(cos(current_euler.x())*cos(current_euler.y()));
    attitude_target.thrust = inverse_thrust_model(a_z,battery_state.voltage,param,thrust_model);
    
    // X Translation Control
    e_x = ref.ref.pose.position.x - current_pose.pose.position.x;
    e_dx = ref.ref.twist.linear.x - current_twist.twist.linear.x;
    s_x = param.k_xe*e_x + e_dx;
    u_x = (param.k_xe*e_dx+ref.ref.accel.linear.x+param.k_xs*tanh(param.k_xt*s_x))/a_z;

    // Y Translation Control
    e_y = ref.ref.pose.position.y - current_pose.pose.position.y;
    e_dy = ref.ref.twist.linear.y - current_twist.twist.linear.y;
    s_y = param.k_ye*e_y + e_dy;
    u_y = (param.k_ye*e_dy+ref.ref.accel.linear.y+param.k_ys*tanh(param.k_yt*s_y))/a_z;

    // Calculate Target Eulers
    target_euler.x() = asin(u_x*sin(ref_euler.z()) - u_y*cos(ref_euler.z()));
    target_euler.y() = asin((u_x*cos(ref_euler.z()) + u_y*sin(ref_euler.z()))/cos(target_euler.x()));
    target_euler.z() = ref_euler.z();
    attitude_target.orientation = rpy2q(target_euler);

    if (param.pub_debug){
        pub_debug();
    }
    
    return attitude_target;
}

double SLIDINGMODE::get_hover_thrust(){
    return param.hover_thrust;
}

int SLIDINGMODE::sign(double& value){
    if (value > 0){
        return 1;
    }
    else if (value < 0){
        return -1;
    }
    else{
        return 0;
    }
}