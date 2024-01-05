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
    // Q_noise R_noise init
    
}

Eigen::Vector3d DISTURBANCE_OBSERVER::observe(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist,const mavros_msgs::AttitudeTarget attitude_target){
    // attitude_target.thrust
    // x,y,z,u,v,w for measurement and system states
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

    current_euler = BASE_CONTROLLER::q2rpy(pose.pose.orientation);
    measurement_states.phi = current_euler.x();
    measurement_states.theta = current_euler.y();
    measurement_states.psi = current_euler.z();

    system_states.phi = current_euler.x();
    system_states.theta = current_euler.y();
    system_states.psi = current_euler.z();

    Eigen::Vector3d force_disturbance;
    force_disturbance.x() = system_states.disturbance_x;

    return force_disturbance;
}