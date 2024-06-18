#ifndef BASE_OBSERVER_H
#define BASE_OBSERVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>

class BASE_OBSERVER{
    protected:

    public:
        virtual geometry_msgs::AccelStamped observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const double&) = 0;
};

#endif