#ifndef DISTURBANCE_OBSERVER_H
#define DISTURBANCE_OBSERVER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "airo_control/controller/base_controller.h"
#include <iostream>

class DISTURBANCE_OBSERVER{
    private:

    struct MEASUREMENT_STATES{
        double x,y,z,u,v,w,phi,theta,psi,thrust_x,thrust_y,thrust_z;
    };
    MEASUREMENT_STATES measurement_states;
    
    struct SYSTEM_STATES{
        double x,y,z,u,v,w,phi,theta,psi,disturbance_x,disturbance_y,disturbance_z;
    };
    SYSTEM_STATES system_states;
    
    // Parameters
    double FSM_FREQUENCY,hover_thrust,current_euler,R_POS,R_VEL,R_ATT,R_CONTROL,Q_POS,Q_VEL,Q_ATT,Q_DISTURBANCE;
    double g = 9.81;
    double dt;

    // Weights
    int m = 12;
    int n = 12;
    Eigen::Matrix<double,12,12> Q_noise,R_noise,P0,esti_P;       // Process noise matrix, Measurement noise matrix, Initial covariance, Estimate covariance
    Eigen::Matrix<double,1,12> Q_cov,R_cov;                      // Process noise value, Measurement noise value

    // EKF Parameters
    Eigen::Matrix<double,3,1> input_u;                           // Inputs
    Eigen::Matrix<double,12,1> meas_y;                           // Measurement vector
    Eigen::Matrix<double,12,12> F,H,Kal,P_pred;                  // Jacobian of system dynamics, Jacobian of measurement model, Kalman gain matrix, Predicted covariance 
    Eigen::Matrix<double,12,1> esti_x,x_pred,y_pred,y_err;       // Estimate states, Predicted state, Predicted measurement, Measurement error

    public:
    DISTURBANCE_OBSERVER(ros::NodeHandle&,const double&);
    Eigen::Vector3d observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&,const mavros_msgs::AttitudeTarget);
    void EKF();
    Eigen::MatrixXd RK4(Eigen::MatrixXd x, Eigen::MatrixXd u);                   // EKF predict and update
    Eigen::MatrixXd f(Eigen::MatrixXd x, Eigen::MatrixXd u);                     // system process model
    Eigen::MatrixXd h(Eigen::MatrixXd x);                                 // measurement model
    Eigen::MatrixXd compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u);    // compute Jacobian of system process model
    Eigen::MatrixXd compute_jacobian_H(Eigen::MatrixXd x);                // compute Jacobian of measurement model
};

#endif