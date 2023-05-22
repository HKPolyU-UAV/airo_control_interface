#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <airo_px4/FSMInfo.h>
#include <airo_px4/TakeoffLandTrigger.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pose,takeoff_pose,x_maneuver_pose,y_maneuver_pose,yaw_maneuver_pose;
double mean_thrust, mean_tau_phi, mean_tau_theta, mean_tau_psi, sum_thrust = 0, sum_tau_phi = 0, sum_tau_theta = 0, sum_tau_psi = 0;
std::vector<double> thrust,tau_phi,tau_theta,tau_psi;
bool hover_thrust_id = false;
bool tau_phi_id = false;
bool tau_theta_id = false;
bool tau_psi_id = false;
ros::Time last_state_time;
std::string package_path = ros::package::getPath("airo_px4");
std::string yaml_path = package_path + "/config/fsm_param.yaml";
tf::Quaternion tf_quaternion;

enum State{
    TAKEOFF,
    HOVER,
    X_MANEUVER,
    Y_MANEUVER,
    YAW_MANEUVER,
    LAND
};

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

Eigen::Vector3d q2rpy(geometry_msgs::Quaternion q){
    Eigen::Vector3d euler;
    tf::quaternionMsgToTF(q,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.x(), euler.y(), euler.z());
    return euler;
}

void sync_cb(const geometry_msgs::PoseStamped::ConstPtr& attitude_msg, mavros_msgs::AttitudeTarget::ConstPtr& attitude_target_msg, geometry_msgs::TwistStamped::ConstPtr& angular_rate_msg){
    Eigen::Vector3d current_target_euler = q2rpy(attitude_target_msg->orientation);
    Eigen::Vector3d current_euler = q2rpy(attitude_msg->pose.orientation);
    
    if (tau_phi_id){
        double current_tau_phi = (current_target_euler.x() - current_euler.x())/angular_rate_msg->twist.angular.x;
        tau_phi.push_back(current_tau_phi);
        sum_tau_phi += current_tau_phi;
    }else if (tau_theta_id){
        double current_tau_theta = (current_target_euler.y() - current_euler.y())/angular_rate_msg->twist.angular.y;
        tau_theta.push_back(current_tau_theta);
        sum_tau_theta += current_tau_theta;
    }else if (tau_psi_id){
        double current_tau_psi = (current_target_euler.z() - current_euler.z())/angular_rate_msg->twist.angular.z;
        tau_psi.push_back(current_tau_psi);
        sum_tau_psi += current_tau_psi;
    }
}

void target_actuator_control_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg){
    if (hover_thrust_id){
        thrust.push_back(msg->controls[3]);
        sum_thrust += msg->controls[3];
    }
}

bool target_reached(const geometry_msgs::PoseStamped& msg){
    return sqrt(pow(msg.pose.position.x - local_pose.pose.position.x,2)+pow(msg.pose.position.y - local_pose.pose.position.y,2)
    +pow(msg.pose.position.z - local_pose.pose.position.z,2)) < 0.25;
}

void update_x_maneuver(){
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "system_identification");
    ros::NodeHandle nh;
    ros::Rate rate(40.0);
    State state = TAKEOFF;
    YAML::Node yaml_config =  YAML::LoadFile(yaml_path);
    std::ofstream yaml_file(yaml_path);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber target_actuator_control_sub = nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control",100,target_actuator_control_cb);
    ros::Publisher command_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    message_filters::Subscriber<geometry_msgs::PoseStamped> attitude_sub(nh,"/mavros/local_position/pose",10);
    message_filters::Subscriber<mavros_msgs::AttitudeTarget> attitude_target_sub(nh,"/mavros/setpoint_raw/target_attitude",10);
    message_filters::Subscriber<geometry_msgs::TwistStamped> angular_rate_sub(nh,"/mavros/local_position/velocity_local",10);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mavros_msgs::AttitudeTarget, geometry_msgs::TwistStamped> my_sync_policy;

    message_filters::Synchronizer<my_sync_policy> sync(my_sync_policy(10), attitude_sub,attitude_target_sub,angular_rate_sub);
    sync.registerCallback(boost::bind(&sync_cb,_1,_2,_3));

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    takeoff_pose.pose.position.x = 0;
    takeoff_pose.pose.position.y = 0;
    takeoff_pose.pose.position.z = 1;
    takeoff_pose.pose.orientation.w = 1;
    takeoff_pose.pose.orientation.x = 0;
    takeoff_pose.pose.orientation.y = 0;
    takeoff_pose.pose.orientation.z = 0;

    for(int i = 100; ros::ok() && i > 0; --i){
        command_pub.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                    last_request = ros::Time::now();
                    }
                }

                command_pub.publish(takeoff_pose);

                if(target_reached(takeoff_pose)){
                    state = HOVER;
                    last_state_time = ros::Time::now();
                    hover_thrust_id = true;
                }

                break;
            }

            case HOVER:{
                command_pub.publish(takeoff_pose);
                if (ros::Time::now().toSec() - last_state_time.toSec() > 10.0){
                    hover_thrust_id = false;
                    mean_thrust = sum_thrust/thrust.size();
                    yaml_config["hover_thrust"] = mean_thrust;
                    state = X_MANEUVER;
                    tau_phi_id = true;
                    last_state_time = ros::Time::now();
                }
                break;
            }

            case X_MANEUVER:{
                if (ros::Time::now().toSec() - last_state_time.toSec() > 10.0){
                    tau_phi_id = false;
                    mean_tau_phi = sum_tau_psi/tau_phi.size();
                    yaml_config["tau_phi"] = mean_tau_phi;
                    state = Y_MANEUVER;
                    tau_theta_id = true;
                    last_state_time = ros::Time::now();
                    while(ros::ok()){
                        command_pub.publish(takeoff_pose);
                        if (target_reached(takeoff_pose)){
                            break;
                        }
                        ros::spinOnce();
                        ros::Duration(rate).sleep();                        
                    }
                }
                else{
                    update_x_maneuver();
                    command_pub.publish(x_maneuver_pose);
                }
                break;
            }


        }
                    // yaml_file << yaml_config;
                    // yaml_file.close();
        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}