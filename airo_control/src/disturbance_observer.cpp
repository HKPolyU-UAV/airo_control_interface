#include "airo_control/disturbance_observer.h"

DISTURBANCE_OBSERVER::DISTURBANCE_OBSERVER(ros::NodeHandle& nh,const double& HOVER_THRUST){
    // ROS Parameters
    hover_thrust = HOVER_THRUST;
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);
    nh.getParam("airo_control_node/observer/r_pos",R_POS);

    // Weights
    // Q_noise R_noise init
}

Eigen::Vector3d DISTURBANCE_OBSERVER::observe(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist,const mavros_msgs::AttitudeTarget attitude_target){
    // attitude_target.thrust
    measurement_states.x = pose.pose.position.x;

    Eigen::Vector3d force_disturbance;
    force_disturbance.x() = system_states.disturbance_x;

    return force_disturbance;
}