#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_message/FSMInfo.h>
#include <airo_message/TakeoffLandTrigger.h>
#include <airo_message/ReferenceStamped.h>

geometry_msgs::PoseStamped local_pose;
airo_message::FSMInfo fsm_info;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void fsm_info_cb(const airo_message::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    airo_message::ReferenceStamped target_pose_1;
    airo_message::ReferenceStamped target_pose_2;
    airo_message::TakeoffLandTrigger takeoff_land_trigger;
    bool target_1_reached = false;

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_message::FSMInfo>("/airo_control/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_message::ReferenceStamped>("/airo_control/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",10);

    target_pose_1.ref.pose.position.x = 0;
    target_pose_1.ref.pose.position.y = -2.5;
    target_pose_1.ref.pose.position.z = 1.5;
    target_pose_1.ref.pose.orientation.w = 0.9238;
    target_pose_1.ref.pose.orientation.x = 0.0;
    target_pose_1.ref.pose.orientation.y = 0.0;
    target_pose_1.ref.pose.orientation.z = 0.3826;

    target_pose_2.ref.pose.position.x = 0;
    target_pose_2.ref.pose.position.y = 2.5;
    target_pose_2.ref.pose.position.z = 1.5;
    target_pose_2.ref.pose.orientation.w = 1.0;
    target_pose_2.ref.pose.orientation.x = 0.0;
    target_pose_2.ref.pose.orientation.y = 0.0;
    target_pose_2.ref.pose.orientation.z = 0.0;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(fsm_info.is_landed == true){
                    while(ros::ok()){
                        takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                        takeoff_land_trigger.header.stamp = ros::Time::now();
                        takeoff_land_pub.publish(takeoff_land_trigger);
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        if(fsm_info.is_waiting_for_command){
                            state = COMMAND;
                            break;
                        }
                    }
                }
                break;
            }

            case COMMAND:{
                if(fsm_info.is_waiting_for_command){
                    if(!target_1_reached){
                        target_pose_1.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_1);
                        if(abs(local_pose.pose.position.x - target_pose_1.ref.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.ref.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.ref.pose.position.z) < 0.5){
                            target_1_reached = true;
                        }
                    }
                    else{
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        if(abs(local_pose.pose.position.x - target_pose_2.ref.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_2.ref.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_2.ref.pose.position.z) < 0.5){
                            state = LAND;
                        }
                    }
                }
                break;
            }

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}