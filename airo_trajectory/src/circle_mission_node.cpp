#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_control/FSMInfo.h>
#include <airo_control/TakeoffLandTrigger.h>
#include <airo_control/Reference.h>

double current_twist;
double radius = 1.5;
double velocity = 1.0;
double height = 1.0;
double x_origin = 0.5;
double y_origin = 0.5;
double circle_mission_time;
geometry_msgs::PoseStamped local_pose;
airo_control::Reference target_pose;
airo_control::FSMInfo fsm_info;
airo_control::TakeoffLandTrigger takeoff_land_trigger;
ros::Time circle_start_time; 

enum State{
    TAKEOFF,
    TOSTART,
    CIRCLE,
    STOP,
    LAND
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_twist = sqrt(msg->twist.linear.x * msg->twist.linear.x
                       + msg->twist.linear.y * msg->twist.linear.y
                       + msg->twist.linear.z * msg->twist.linear.z);
}

void fsm_info_cb(const airo_control::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

double circle_traj(double time_elapsed, int indicator){ // 1 for x pos, 2 for y pos, 3 for x twist, 4 for y twist
    double ref;
    if (indicator == 1){
        ref = -radius*cos(time_elapsed*velocity/radius) + x_origin;
    }
    else if (indicator == 2){
        ref = -radius*sin(time_elapsed*velocity/radius) + y_origin;
    }
    else if (indicator == 3){
        ref = velocity*sin(time_elapsed*velocity/radius);
    }
    else if (indicator == 4){
        ref = -velocity*cos(time_elapsed*velocity/radius);
    }
    return ref;
}

bool target_reached(const geometry_msgs::Point& msg){
    return sqrt(pow(msg.x - local_pose.pose.position.x,2)+pow(msg.y - local_pose.pose.position.y,2)
    +pow(msg.z - local_pose.pose.position.z,2)) < 1.0;
}

void update_circle_traj(double time_elapsed){
    double yaw_angle = atan2(y_origin - circle_traj(time_elapsed,2), x_origin - circle_traj(time_elapsed,1));
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw_angle);
    for (int i = 0; i < 41; i++){
        target_pose.ref_pose[i].position.x = circle_traj(time_elapsed,1);
        target_pose.ref_pose[i].position.y = circle_traj(time_elapsed,2);
        target_pose.ref_pose[i].position.z = height;
        target_pose.ref_pose[i].orientation = quaternion;
    }

    for (int i = 0; i < 41; i++){
        target_pose.ref_twist[i].linear.x = circle_traj(time_elapsed,3);
        target_pose.ref_twist[i].linear.y = circle_traj(time_elapsed,4);
        target_pose.ref_twist[i].linear.z = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(40.0);
    State state = TAKEOFF;

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber local_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",100,twist_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_control::FSMInfo>("/airo_control/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_control::Reference>("/airo_control/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_control::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",10);

    target_pose.ref_pose.resize(41);
    target_pose.ref_twist.resize(41);

    update_circle_traj(0.0);

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
                            state = TOSTART;
                            break;
                        }
                    }
                }
                break;
            }

            case TOSTART:{
                if(fsm_info.is_waiting_for_command){
                    if(target_reached(target_pose.ref_pose[0].position) && current_twist < 0.25){
                        state = CIRCLE;
                        circle_start_time = ros::Time::now();
                        break;
                    }
                    else{
                        target_pose.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose);
                    }
                }
                break;
            }

            case CIRCLE:{
                if (fsm_info.is_waiting_for_command){
                    circle_mission_time = (ros::Time::now() - circle_start_time).toSec();
                    if (circle_mission_time > 40.0){
                        state = STOP;
                        break;
                    }
                    else{
                        update_circle_traj(circle_mission_time);
                        target_pose.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose);
                    }
                }
                break;
            }

            case STOP:{
                if(fsm_info.is_waiting_for_command){
                    if(target_reached(target_pose.ref_pose[0].position) && current_twist < 0.25){
                        state = LAND;
                        break;
                    }
                    else{
                        target_pose.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose);
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