#include <ros/ros.h>

#include "airo_control/airo_control_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airo_control_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    AIRO_CONTROL_FSM fsm(nh);

    while(ros::ok()){
        fsm.process();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}