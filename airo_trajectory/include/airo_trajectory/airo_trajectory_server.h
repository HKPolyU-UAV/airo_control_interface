#ifndef AIRO_TRAJECTORY_UTILS_H
#define AIRO_TRAJECTORY_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_control/FSMInfo.h>
#include <airo_control/TakeoffLandTrigger.h>
#include <airo_control/Reference.h>
#include <airo_control/ReferencePreview.h>

class AIRO_TRAJECTORY_SERVER{
    private:

    ros::Subscriber local_pose_sub, local_twist_sub, fsm_info_sub;
    ros::Publisher command_pub, command_preview_pub, takeoff_land_pub;
    airo_control::FSMInfo fsm_info;
    geometry_msgs::PoseStamped local_pose;
    double current_twist;
    std::string POSE_TOPIC, TWIST_TOPIC;
    const int preview_size = 41;

    public:

    AIRO_TRAJECTORY_SERVER(ros::NodeHandle&);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
    void fsm_info_cb(const airo_control::FSMInfo::ConstPtr&);
    void pose_cmd(const geometry_msgs::Point&, const double& yaw_angle = 0.0);
    void pose_cmd(const geometry_msgs::Point&, const geometry_msgs::Twist&, const double& yaw_angle = 0.0);
    void pose_cmd(const geometry_msgs::Point&, const geometry_msgs::Twist&, const geometry_msgs::Accel&, const double& yaw_angle = 0.0);
    bool target_reached(const geometry_msgs::Point&);
    geometry_msgs::Quaternion yaw_to_quaternion(double);
    double quaternion_to_yaw(const geometry_msgs::Quaternion&);
    void file_traj_init(const std::string&, std::vector<std::vector<double>>&);
    geometry_msgs::Point get_start_point(const std::vector<std::vector<double>>&);
    geometry_msgs::Point get_end_point(const std::vector<std::vector<double>>&);
    bool file_cmd(const std::vector<std::vector<double>>& traj, int&);
    void assign_position(const std::vector<std::vector<double>>&, airo_control::ReferencePreview&);
    void assign_twist(const std::vector<std::vector<double>>&, airo_control::ReferencePreview&);
    void assign_accel(const std::vector<std::vector<double>>&, airo_control::ReferencePreview&);
    void assign_yaw(const std::vector<std::vector<double>>&, airo_control::ReferencePreview&);
    bool takeoff();
    bool land();
};

#endif