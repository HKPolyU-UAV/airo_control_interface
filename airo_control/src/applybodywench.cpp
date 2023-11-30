#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include "airo_control/airo_control_fsm.h"

ros::ServiceClient body_wrench_client;

struct wrench{
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
    };

wrench applied_wrench;

gazebo_msgs::ApplyBodyWrench wrench; 

void applyDisturbance()
{
    wrench.request.body_name = "iris/base_link";
    wrench.request.reference_frame = "world";
    wrench.request.wrench.force.x = applied_wrench.fx;
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

    
    // Main loop
    while (ros::ok())
    {
        applied_wrench.fx = 10;
        // Call the applyDisturbance function
        applyDisturbance();

        // Handle callbacks and process messages
        ros::spinOnce();

        // Perform any additional processing or logic here

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;

}


