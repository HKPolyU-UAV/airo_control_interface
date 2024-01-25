#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <airo_message/Reference.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <airo_control/disturbance_observer.h>

class BASE_CONTROLLER{
protected:
    struct Param{
        bool pub_debug;
        double hover_thrust;
        double tau_phi;
        double tau_theta;
    };

    Param param;
    // Eigen::Vector3d ref_euler,current_euler,target_euler;
    mavros_msgs::AttitudeTarget attitude_target;

    Eigen::Vector3d q2rpy(const geometry_msgs::Quaternion&);  // Quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const Eigen::Vector3d&);  // euler angle to quaternion

public:
    Eigen::Vector3d ref_euler,current_euler,target_euler;
    virtual void pub_debug() = 0;
    virtual double get_hover_thrust() = 0;
    virtual mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&) = 0;
    virtual mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&, const geometry_msgs::Vector3Stamped&) = 0;
};

#endif