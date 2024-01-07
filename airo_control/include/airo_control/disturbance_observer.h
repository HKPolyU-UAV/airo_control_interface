#ifndef DISTURBANCE_OBSERVER_H
#define DISTURBANCE_OBSERVER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "airo_control/controller/base_controller.h"

class DISTURBANCE_OBSERVER{
    private:

    double g = 9.81;
    double dt = 1 / ;

    struct MEASUREMENT_STATES{
        double x,y,z,u,v,w,phi,theta,psi,thrust_x,thrust_y,thrust_z;
    };
    MEASUREMENT_STATES measurement_states;
    
    struct SYSTEM_STATES{
        double x,y,z,u,v,w,phi,theta,psi,disturbance_x,disturbance_y,disturbance_z;
    };
    SYSTEM_STATES system_states;
    
    // Parameters
    double hover_thrust,R_POS,R_VEL,R_ATT,R_CONTROL,Q_POS,Q_VEL,Q_ATT,Q_DISTURBANCE;

    // Weights
    Eigen::Matrix<double,14,14> Q_noise,R_noise;

    public:
    DISTURBANCE_OBSERVER(ros::NodeHandle&,const double&);
    Eigen::Vector3d observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&,const mavros_msgs::AttitudeTarget);
};

#endif