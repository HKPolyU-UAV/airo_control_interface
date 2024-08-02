#ifndef EKF_H
#define EKF_H

#include <geometry_msgs/Vector3Stamped.h>
#include "airo_control/observer/base_observer.h"
#include <eigen3/Eigen/Dense>


class EKF : public BASE_OBSERVER{
    protected:

    struct MEASUREMENT_STATES{
        double x,y,z,u,v,w;                                // thrust_x_b = thrust_y_b = 0
    };
    MEASUREMENT_STATES measurement_states;

    struct SYSTEM_STATES{
        double x,y,z,u,v,w,disturbance_x,disturbance_y,disturbance_z;
    };
    SYSTEM_STATES system_states;
    
    struct ACCEL{
        double x_b,y_b,z_b;                         // b: body frame; w: world frame
    };
    ACCEL accel;

    // Parameters
    double FSM_FREQUENCY,R_POS_X,R_POS_Y,R_POS_Z,R_VEL_X,R_VEL_Y,R_VEL_Z,R_CONTROL_X,R_CONTROL_Y,R_CONTROL_Z,Q_POS_X,Q_POS_Y,Q_POS_Z,Q_VEL_X,Q_VEL_Y,Q_VEL_Z,Q_DISTURBANCE_X,Q_DISTURBANCE_Y,Q_DISTURBANCE_Z;
    double dt,safe_dis;
    int cout_counter = 0;
    double thrust_per_mass;
    
    // Weights
    int m = 9;
    int n = 9;
    Eigen::Matrix<double,9,9> Q_noise,R_noise,P0,esti_P;         // Process noise matrix, Measurement noise matrix, Initial covariance, Estimate covariance
    Eigen::Matrix<double,1,9> Q_cov,R_cov;                       // Process noise value, Measurement noise value

    // EKF Parameters
    Eigen::Matrix<double,3,1> input_u;                           // Inputs
    Eigen::Matrix<double,9,1> meas_y;                            // Measurement vector
    Eigen::Matrix<double,9,1> esti_x;                            // Estimate states

    public:
        EKF(ros::NodeHandle&);
        geometry_msgs::AccelStamped observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const double&);
        Eigen::MatrixXd RK4(Eigen::MatrixXd x, Eigen::MatrixXd u);                   // EKF predict and update
        Eigen::MatrixXd f(Eigen::MatrixXd x, Eigen::MatrixXd u);                     // system process model
        Eigen::MatrixXd h(Eigen::MatrixXd x);                                        // measurement model
        Eigen::MatrixXd compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u);    // compute Jacobian of system process model
        Eigen::MatrixXd compute_jacobian_H(Eigen::MatrixXd x); 
        Eigen::Matrix3d q2ROT(const geometry_msgs::Quaternion); 
};

#endif
