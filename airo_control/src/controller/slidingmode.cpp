#include "airo_control/controller/slidingmode.h"

SLIDINGMODE::SLIDINGMODE(ros::NodeHandle& nh){
    // Get Parameters
    nh.getParam("airo_control_node/slidingmode/hover_thrust",param.hover_thrust);
    nh.getParam("airo_control_node/slidingmode/pub_debug",param.pub_debug);
    nh.getParam("airo_control_node/slidingmode/k_xe",param.k_xe);
    nh.getParam("airo_control_node/slidingmode/k_xs",param.k_xs);
    nh.getParam("airo_control_node/slidingmode/k_xt",param.k_xt);
    nh.getParam("airo_control_node/slidingmode/k_ye",param.k_ye);
    nh.getParam("airo_control_node/slidingmode/k_ys",param.k_ys);
    nh.getParam("airo_control_node/slidingmode/k_yt",param.k_yt);
    nh.getParam("airo_control_node/slidingmode/k_ze",param.k_ze);
    nh.getParam("airo_control_node/slidingmode/k_zs",param.k_zs);
    nh.getParam("airo_control_node/slidingmode/k_zt",param.k_zt);

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

mavros_msgs::AttitudeTarget SLIDINGMODE::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::Reference& ref){
    geometry_msgs::Vector3Stamped force_disturbance;
    return SLIDINGMODE::solve(current_pose,current_twist,current_accel,ref,force_disturbance);
}

mavros_msgs::AttitudeTarget SLIDINGMODE::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::Reference& ref,const geometry_msgs::Vector3Stamped& force_disturbance){  
    current_euler = q2rpy(current_pose.pose.orientation);
    ref_euler = q2rpy(ref.ref_pose.orientation);

    // Altitude Control
    e_z = ref.ref_pose.position.z - current_pose.pose.position.z;
    e_dz= ref.ref_twist.linear.z - current_twist.twist.linear.z;
    s_z = param.k_ze * e_z + e_dz;
    attitude_target.thrust = param.hover_thrust/(g*cos(current_euler.x())*cos(current_euler.y())) * (param.k_ze*e_dz+ref.ref_accel.linear.z+g+param.k_zs*tanh(param.k_zt*s_z));
    if (attitude_target.thrust > 1.0){
        attitude_target.thrust = 1.0;
    }
    else if (attitude_target.thrust < 0.0){
        attitude_target.thrust = 0.0;
    }

    // X Translation Control
    e_x = ref.ref_pose.position.x - current_pose.pose.position.x;
    e_dx = ref.ref_twist.linear.x - current_twist.twist.linear.x;
    s_x = param.k_xe*e_x + e_dx;
    u_x = param.hover_thrust/(attitude_target.thrust*g)*(param.k_xe*e_dx+ref.ref_accel.linear.x+param.k_xs*tanh(param.k_xt*s_x));

    // Y Translation Control
    e_y = ref.ref_pose.position.y - current_pose.pose.position.y;
    e_dy = ref.ref_twist.linear.y - current_twist.twist.linear.y;
    s_y = param.k_ye*e_y + e_dy;
    u_y = param.hover_thrust/(attitude_target.thrust*g)*(param.k_ye*e_dy+ref.ref_accel.linear.y+param.k_ys*tanh(param.k_yt*s_y));

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