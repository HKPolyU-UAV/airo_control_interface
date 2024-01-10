#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>
#include "airo_control/airo_control_fsm.h"

ros::ServiceClient body_wrench_client;

struct WRENCH{
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
};

WRENCH applied_wrench;

gazebo_msgs::ApplyBodyWrench wrench; 


void applyDisturbance()
{
    wrench.request.body_name = "iris::base_link";
    wrench.request.reference_frame = "world"; //NED
    wrench.request.wrench.force.x = applied_wrench.fx;
    wrench.request.wrench.force.y = applied_wrench.fy;
    wrench.request.wrench.force.z = -applied_wrench.fz; //as +ve = Down
    wrench.request.reference_point.x = 0.0;
    wrench.request.reference_point.y = 0.0;
    wrench.request.reference_point.z = 0.0;
    wrench.request.start_time = ros::Time::now();
    wrench.request.duration = ros::Duration(1000);  // Duration of the disturbance
    body_wrench_client.call(wrench);

    std::cout<<"------------------ applied disturbances xyz ------------------"<<std::endl;
    std::cout<<"applied_wrench_fx: "<<wrench.request.wrench.force.x<<" N"<<std::endl;
    std::cout<<"applied_wrench_fy: "<<wrench.request.wrench.force.y <<" N"<<std::endl;
    std::cout<<"applied_wrench_fz: "<<wrench.request.wrench.force.z<<" N"<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disturbance");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    // Initialize the body wrench service client
    body_wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    
    // Main loop
    while (ros::ok())
    {
        applied_wrench.fx = 1.5; //unit: Newtons
        applied_wrench.fy = 2; //unit: Newtons
        applied_wrench.fz = 1; //unit: Newtons

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


