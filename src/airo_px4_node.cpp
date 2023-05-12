#include <ros/ros.h>

#include "airo_px4/airo_px4_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airo_px4_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);

    AIRO_PX4_FSM fsm(nh);

    while(ros::ok()){
        fsm.process();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}