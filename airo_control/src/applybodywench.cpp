#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include "airo_control/airo_control_fsm.h"

ros::ServiceClient body_wrench_client;

void applyDisturbance()
{
    gazebo_msgs::ApplyBodyWrench wrench;
    wrench.request.body_name = "uav::base_link";
    wrench.request.reference_frame = "world";
    wrench.request.wrench.force.x = 10;
    wrench.request.wrench.force.y = 0.0;
    wrench.request.wrench.force.z = 0.0;
    wrench.request.reference_point.x = 0.0;
    wrench.request.reference_point.y = 0.0;
    wrench.request.reference_point.z = 0.0;
    wrench.request.start_time = ros::Time::now();
    wrench.request.duration = ros::Duration(1000);  // Duration of the disturbance
    body_wrench_client.call(wrench);
    std::cout<<"call client"<<std::endl;

    // Call the service to apply the body wrench
    if (body_wrench_client.call(wrench))
    {
        ROS_INFO("Applied disturbance force along x-axis");
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/apply_body_wrench");
        return 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbance");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    // Initialize the body wrench service client
    body_wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    

    std::cout<<"Initialize the body wrench service client"<<std::endl;

    // Apply disturbance after 1 second
    ros::Duration(1.0).sleep();
    std::cout<<"wait 1 second"<<std::endl;
    applyDisturbance();
    std::cout<<"apply disturbance"<<std::endl;
    // Spin the ROS node
    ros::spin();

    return 0;
}


