#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <airo_message/Reference.h>
#include <eigen3/Eigen/Dense>

class BASE_CONTROLLER{
protected:
    struct Param{
        double hover_thrust;
    };
    Param param;
    Eigen::Vector3d q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Eigen::Vector3d&);

public:
    double get_hover_thrust();
    virtual mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&) = 0;
};

#endif