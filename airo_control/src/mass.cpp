#include <ros/ros.h>
#include <gazebo_msgs/GetLinkProperties.h>

ros::ServiceClient client;
gazebo_msgs::GetLinkProperties srv;

double getBodyMass()
{
    if (client.call(srv))
    {
        return srv.response.mass;
    }
    else
    {
        ROS_ERROR_STREAM("Failed to call service /gazebo/get_link_properties");
        return 0.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_body_mass_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    client = nh.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
    srv.request.link_name = "iris::base_link";  // Replace with the actual body name of your drone

    double mass = getBodyMass();

    ROS_INFO_STREAM("Mass is: " << mass << " kg");

    return 0;
}