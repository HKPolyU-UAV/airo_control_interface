#include "airo_px4/quadrotor_mpc.h"
#include <airo_px4/Reference.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lala_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);

    QUADROTOR_MPC controller;

    airo_px4::Reference target_pose;
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::TwistStamped local_twist;
    QUADROTOR_MPC::SolverParam mpc_param;
    mavros_msgs::AttitudeTarget attitude_target;

    target_pose.ref_pose.resize(41);
    target_pose.ref_twist.resize(41);

    for (int i = 0; i < 41; i++){
        target_pose.ref_pose[i].position.x = 0.0;
        target_pose.ref_pose[i].position.y = 0.0;
        target_pose.ref_pose[i].position.z = 1.0;
        target_pose.ref_pose[i].orientation.w = 1.0;
        target_pose.ref_pose[i].orientation.x = 0.0;
        target_pose.ref_pose[i].orientation.y = 0.0;
        target_pose.ref_pose[i].orientation.z = 0.0;
    }

    local_pose.pose.position.x = 0;
    local_pose.pose.position.y = 0;
    local_pose.pose.position.z = 1;
    local_pose.pose.orientation.w = 1;
    local_pose.pose.orientation.x = 0;
    local_pose.pose.orientation.y = 0;
    local_pose.pose.orientation.z = 0;

    local_twist.twist.linear.x = 0;
    local_twist.twist.linear.y = 0;
    local_twist.twist.linear.z = 0;

    mpc_param.hover_thrust = 0.56;
    mpc_param.tau_phi = 0.2;
    mpc_param.tau_theta = 0.2;
    mpc_param.tau_psi = 0.2;

    while(ros::ok()){
        attitude_target = controller.solve(local_pose,local_twist,target_pose,mpc_param);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}