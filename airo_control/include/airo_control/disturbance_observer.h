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
    double dt = 1 / ; // dt = 0.1

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
    int m = 14;
    Eigen::Matrix<double,14,14> Q_noise,R_noise,P0, esti_P;         // Process noise matrix, Measurement noise matrix, Initial covariance, Estimate covariance
    Eigen::Matrix<double,3,1> meas_u;                               // Inputs
    Eigen::Matrix<double,14,1> meas_y                               // Measurement vector
    Eigen::Matrix<double,1,14> Q_cov;                               // Process noise value

    public:
    DISTURBANCE_OBSERVER(ros::NodeHandle&,const double&);
    Eigen::Vector3d observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&,const mavros_msgs::AttitudeTarget);
    void EKF();
    MatrixXd RK4(MatrixXd x, MatrixXd u);                   // EKF predict and update
    MatrixXd f(MatrixXd x, MatrixXd u);                     // system process model
    MatrixXd h(MatrixXd x);                                 // measurement model
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);    // compute Jacobian of system process model
    MatrixXd compute_jacobian_H(MatrixXd x);                // compute Jacobian of measurement model
};

#endif