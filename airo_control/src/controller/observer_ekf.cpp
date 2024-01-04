#include "airo_control/controller/observer_ekf.h"

OBSERVER_EKF::OBSERVER_EKF(ros::NodeHandle& nh){
    // Get EKF parameters
    nh.getParam("airo_control_node/observer_ekf/hover_thrust", param.hover_thrust);
    nh.getParam("airo_control_node/observer_ekf/tau_phi", param.tau_phi);
    nh.getParam("airo_control_node/observer_ekf/tau_theta", param.tau_theta);
    nh.getParam("airo_control_node/observer_ekf/disturbance_x", solverparam.disturbance_x);
    nh.getParam("airo_control_node/observer_ekf/disturbance_y", solverparam.disturbance_y);
    nh.getParam("airo_control_node/observer_ekf/disturbance_z", solverparam.disturbance_z);

}
