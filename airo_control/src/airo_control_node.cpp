#include <ros/ros.h>

#include "airo_control/airo_control_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airo_control_node");
    ros::NodeHandle nh;
    int FSM_FREQUENCY;
    nh.getParam("airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);
    ros::Rate loop_rate(FSM_FREQUENCY);
    AIRO_CONTROL_FSM fsm(nh);

    while(ros::ok()){
        fsm.process(); 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}