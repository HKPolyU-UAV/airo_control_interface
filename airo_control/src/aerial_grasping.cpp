#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_message/FSMInfo.h>
#include <airo_message/TakeoffLandTrigger.h>
#include <airo_message/Reference.h>
#include <iostream>
#include <istream>
#include <sstream>
#include <fstream>
#include <string>
#include "std_msgs/Float32.h"

geometry_msgs::PoseStamped local_pose, object_pose, current_object_pose;
airo_message::FSMInfo fsm_info;
int cnt = 0;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

airo_message::Reference target_pose_1;
airo_message::Reference target_pose_2;
airo_message::Reference target_pose_3;

airo_message::TakeoffLandTrigger takeoff_land_trigger;
bool target_1_reached = false;
bool target_2_reached = false;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void fsm_info_cb(const airo_message::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

void object_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& object_pose){
    current_object_pose.pose.position.x = object_pose->pose.position.x;
    current_object_pose.pose.position.y = object_pose->pose.position.y;
    current_object_pose.pose.position.z = object_pose->pose.position.z;   
}

void setDutyCycle(int dutyCycle) {
    std::ofstream pwmFile;
    pwmFile.open("/sys/class/pwm/pwmchip4/pwm1/duty_cycle");
    pwmFile << dutyCycle;
    pwmFile.close();
}

void flightCommandCallback(const std_msgs::Float32::ConstPtr& msg) {
  // Extract the desired duty cycle from the message
  float dutyCycle = msg->data;

  // Update the duty cycle
  setDutyCycle(dutyCycle);
}

void datalogger(int i){
    if (i == 1){
    std::ofstream save("/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/tracking_errors/tracking1.csv", std::ios::app);
    save<<std::setprecision(20)<<ros::Time::now().toSec()<<
        ","<<local_pose.pose.position.x <<","<< target_pose_1.ref_pose.position.x
        <<","<<local_pose.pose.position.y <<","<< target_pose_1.ref_pose.position.y
        <<","<<local_pose.pose.position.z <<","<< target_pose_1.ref_pose.position.z<<std::endl;
    save.close();
    }
    if (i == 2){
    std::ofstream save("/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/tracking_errors/tracking2.csv", std::ios::app);
    save<<std::setprecision(20)<<ros::Time::now().toSec()<<
        ","<<local_pose.pose.position.x <<","<< target_pose_2.ref_pose.position.x
        <<","<<local_pose.pose.position.y <<","<< target_pose_2.ref_pose.position.y
        <<","<<local_pose.pose.position.z <<","<< target_pose_2.ref_pose.position.z<<std::endl;
    save.close();
    }
    if (i == 3){
    std::ofstream save("/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/tracking_errors/tracking3.csv", std::ios::app);
    save<<std::setprecision(20)<<ros::Time::now().toSec()<<
        ","<<local_pose.pose.position.x <<","<< target_pose_3.ref_pose.position.x
        <<","<<local_pose.pose.position.y <<","<< target_pose_3.ref_pose.position.y
        <<","<<local_pose.pose.position.z <<","<< target_pose_3.ref_pose.position.z<<std::endl;
    save.close();
    }
    if (i == 4){
    std::ofstream save("/home/athena/airo_control_interface_ws/src/airo_control_interface/airo_control/src/tracking_errors/tracking4.csv", std::ios::app);
    save<<std::setprecision(20)<<ros::Time::now().toSec()<<
        ","<<local_pose.pose.position.x <<","<< target_pose_3.ref_pose.position.x
        <<","<<local_pose.pose.position.y <<","<< target_pose_3.ref_pose.position.y
        <<","<<local_pose.pose.position.z <<","<< "0"<<std::endl;
    save.close();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    // Subscribe to the MAVROS topic for flight commands
    ros::Subscriber sub = nh.subscribe("pwm_commands", 1, flightCommandCallback);
    // Enable the PWM controller
    std::ofstream exportFile;
    exportFile.open("/sys/class/pwm/pwmchip4/export");
    exportFile << "1";
    exportFile.close();

    // Set the period time
    std::ofstream periodFile;
    periodFile.open("/sys/class/pwm/pwmchip4/pwm1/period");
    periodFile << "2000000";
    periodFile.close();

    // Set the duty cycle to 800000 (gripper initially opens)
    setDutyCycle(800000);

    // Enable output from the PWM pin
    std::ofstream enableFile;
    enableFile.open("/sys/class/pwm/pwmchip4/pwm1/enable");
    enableFile << "1";
    enableFile.close();

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber object_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_sav_bottle/pose", 10, object_pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_message::FSMInfo>("/airo_control/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_message::Reference>("/airo_control/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",10);

    target_pose_1.ref_pose.position.x = current_object_pose.pose.position.x;
    target_pose_1.ref_pose.position.y = current_object_pose.pose.position.y-0.1;
    target_pose_1.ref_pose.position.z = 0.7;
    target_pose_1.ref_pose.orientation.w = 1.0;
    target_pose_1.ref_pose.orientation.x = 0.0;
    target_pose_1.ref_pose.orientation.y = 0.0;
    target_pose_1.ref_pose.orientation.z = 0.0;

    target_pose_2.ref_pose.position.x = -1;
    target_pose_2.ref_pose.position.y = 0;
    target_pose_2.ref_pose.position.z = 1.5;
    target_pose_2.ref_pose.orientation.w = 1.0;
    target_pose_2.ref_pose.orientation.x = 0.0;
    target_pose_2.ref_pose.orientation.y = 0.0;
    target_pose_2.ref_pose.orientation.z = 0.0;

    target_pose_3.ref_pose.position.x = -1.5;
    target_pose_3.ref_pose.position.y = 0;
    target_pose_3.ref_pose.position.z = 0.6;
    target_pose_3.ref_pose.orientation.w = 1.0;
    target_pose_3.ref_pose.orientation.x = 0.0;
    target_pose_3.ref_pose.orientation.y = 0.0;
    target_pose_3.ref_pose.orientation.z = 0.0;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(fsm_info.is_landed == true){
                    //Update the duty cycle to 1200000 (Gripper fully opens)
                    setDutyCycle(1200000);
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
                        datalogger(1);
                        std::cout<<"------- current xyz -------"<<std::endl;
                        std::cout<<"local_pose.x: "<< local_pose.pose.position.x<<std::endl;
                        std::cout<<"local_pose.y: "<< local_pose.pose.position.y<<std::endl;
                        std::cout<<"local_pose.z: "<< local_pose.pose.position.z<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_1.ref_pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_1.ref_pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_1.ref_pose.position.z) < 0.5){
                            std::cout<<"--- ---- current tracking error -------"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_1.ref_pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_1.ref_pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_1.ref_pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            if (cnt<150){
                                //Update the duty cycle to 1600000 (Gripper fully closes)
                                setDutyCycle(1600000);
                                std::cout<<"gasping"<<std::endl;
                                cnt++;
                            }
                            else{
                                cnt = 0;
                                target_1_reached = true;

                            }   
                        }
                    }
                    if(target_1_reached){
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        datalogger(2);
                        std::cout<<"--- ---- current xyz -------"<<std::endl;
                        std::cout<<"local_pose.x: "<< local_pose.pose.position.x<<std::endl;
                        std::cout<<"local_pose.y: "<< local_pose.pose.position.y<<std::endl;
                        std::cout<<"local_pose.z: "<< local_pose.pose.position.z<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_2.ref_pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_2.ref_pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_2.ref_pose.position.z) < 0.5){
                            std::cout<<"--- ---- current tracking error -------"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_2.ref_pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_2.ref_pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_2.ref_pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            if (cnt<100){
                                cnt++;
                                //Update the duty cycle to 1600000 (Gripper fully closes)
                                setDutyCycle(1600000);
                                std::cout<<"holding and lifting object"<<std::endl;

                            }
                            else{
                                cnt = 0;
                                // state = LAND;
                                target_2_reached = true;
                            }
                        }
                    }
                    if (target_2_reached){
                        target_pose_3.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_3);
                        datalogger(3);
                        std::cout<<"--- ---- current xyz -------"<<std::endl;
                        std::cout<<"local_pose.x: "<< local_pose.pose.position.x<<std::endl;
                        std::cout<<"local_pose.y: "<< local_pose.pose.position.y<<std::endl;
                        std::cout<<"local_pose.z: "<< local_pose.pose.position.z<<std::endl;
                        if(abs(local_pose.pose.position.x - target_pose_3.ref_pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_3.ref_pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_3.ref_pose.position.z) < 0.5){
                            std::cout<<"--- ---- current tracking error -------"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_3.ref_pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_3.ref_pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_3.ref_pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            if (cnt<100){
                                cnt++;
                                //Update the duty cycle to 1200000 (Gripper fully opens)
                                setDutyCycle(1200000);
                                std::cout<<"releasing object"<<std::endl;

                            }
                            else{
                                cnt = 0;
                                state = LAND;
                            }
                        }
                    }
                }
                break;
            }

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    // Update the duty cycle to 1200000 (Gripper fully opens)
                    setDutyCycle(1200000);
                    datalogger(4);
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                    std::cout<<"landing"<<std::endl;
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}
