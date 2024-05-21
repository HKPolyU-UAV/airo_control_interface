#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/BatteryState.h>
#include <airo_message/ReferenceStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>

class BASE_CONTROLLER{
protected:
    struct Param{
        bool pub_debug = false;
        bool enable_thrust_model = false;
        double hover_thrust = 0.0;
    };

    struct ThrustModel{
        float mass;
        float K1;
        float K2;
        float K3;
    };

    Param param;
    ThrustModel thrust_model;
    double g = 9.80665;
    Eigen::Vector3d ref_euler,current_euler,target_euler;
    mavros_msgs::AttitudeTarget attitude_target;
    
public:
    virtual void pub_debug() = 0;
    virtual double get_hover_thrust() = 0;
    virtual mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::ReferenceStamped&, const sensor_msgs::BatteryState&) = 0;
    Eigen::Vector3d q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Eigen::Vector3d&);
    float inverse_thrust_model(const double& a_z,const float& voltage,const Param& param,const ThrustModel& thrust_model);

};

#endif