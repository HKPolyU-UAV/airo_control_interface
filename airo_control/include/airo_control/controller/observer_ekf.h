#ifndef OBSERVER_EKF_H
#define OBSERVER_EKF_H

#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>

#include "acados_solver_quadrotor.h"

#include "airo_control/controller/base_controller.h"
#include "airo_message/ReferencePreview.h"

using namespace Eigen;

class OBSERVER_EKF: public BASE_CONTROLLER{
    private:
        struct Param: public BASE_CONTROLLER::Param{
            double tau_phi;
            double tau_theta;
            double tau_psi;
        };

        double g = 9.81;
        double dt = 0.05;

        enum SystemStates{
            x = 0,
            y = 1, 
            z = 2,
            u = 3, 
            v = 4,
            w = 5,
            phi = 6,
            theta = 7,
            psi = 8,
            p = 9,
            q = 10,
        };

        enum ControlInputs{
            thrust = 0,
            phi_cmd = 1,
            theta_cmd = 2,
            psi_cmd = 3,
        };

        // struct SolverInput{
        //     double x0[QUADROTOR_NX];
        //     double yref[QUADROTOR_N+1][QUADROTOR_NY];
        // };

        // struct SolverOutput{
        //     double u0[QUADROTOR_NU];
        //     double x1[QUADROTOR_NX];
        //     double status,kkt_res, cpu_time;
        // };

        struct Euler{
            double phi;
            double theta;
            double psi;
        };

        struct Pos{
            double x;
            double y;
            double z;
        };

        struct Vel{
            double u;  // dx
            double v;  // dy
            double w;  // dz
            double p;  // dphi
            double q;  // dtheta
            double r;  // dpsi
        };

        struct Acc{
            double x;  // du
            double y;  // dv
            double z;  // dw
        };

        struct SolverParam{
            double disturbance_x;
            double disturbance_y;
            double disturbance_z;
        };

        struct Thrust{
            double thrust0;  // U1 in du
            double thrust1;  // U1 in dv
            double thrust2;  // U1 in dw
        };

        // ROS message variables
        Euler local_euler;
        Pos local_pos;
        Vel local_vel;
        Vel pre_wf_vel;
        Acc wf_acc;
        Thrust current_thrust;

        // Messages
        geometry_msgs::PoseStamped esti_pose;
        geometry_msgs::TwistStamped esti_twist;
        geometry_msgs::PoseStamped esti_disturbance;

        // Dynamics parameters
        // Matrix<double,1,6> M_values;
        // Matrix<double,6,6> M;                // Mass matrix
        // Matrix<double,6,6> invM;             // Inverse mass matrix
        Matrix<double,3,1> v_linear_wf;         // Linear velocity in world frame
        // Matrix<double,2,1> v_angular_body;        // Angular velocity in body frame
        // Matrix<double,3,3> R_ib;             // Rotation matrix for linear from inertial to body frame
        // Matrix<double,3,3> T_ib;             // Rotation matrix for angular from inertial to body frame
        
        // EKF parameters
        Matrix<double,6,1> wf_disturbance;          // World frame disturbance
        Matrix<double,3,1> meas_u;                  // Inputs
        int n = 14;                                 // State dimension
        int m = 14;                                 // Measurement dimension
        Matrix<double,14,1> meas_y;                 // Measurement vector
        MatrixXd P0 = MatrixXd::Identity(m,m);      // Initial covariance
        Matrix<double,14,1> esti_x;                 // Estimate states
        Matrix<double,14,14> esti_P;                // Estimate covariance
        Matrix<double,1,14> Q_cov;                  // Process noise value
        Matrix<double,14,14> noise_Q;               // Process noise matrix
        MatrixXd noise_R = MatrixXd::Identity(m,m)*(pow(dt,4)/4);      // Measurement noise matrix
    
        // Time
        ros::Time current_time;

        // ROS subscriber & publisher
        ros::Subscriber pose_sub;
        ros::Publisher thrust0_pub;
        ros::Publisher thrust1_pub;
        ros::Publisher thrust2_pub;

        ros::Publisher ref_pose_pub;
        ros::Publisher error_pose_pub;

        ros::Publisher esti_pose_pub;
        ros::Publisher esti_disturbance_pub;
        ros::Publisher applied_disturbance_pub;
        
        // Acados parameter
        SolverParam solverparam;

        // Other variables
        int cout_counter = 0;
        tf::Quaternion tf_quaternion;


    public:
        Param param;
        Euler q2rpy(const geometry_msgs::Quaternion&);          // quaternion to euler angle
        geometry_msgs::Quaternion rpy2q(const Euler&);          // euler angle to quaternion
        OBSERVER_EKF(ros::NodeHandle&);
        void EKF();
	    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&); // get current position 
        void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&); // get current position 
        void ref_cb(int line_to_read);                                  // fill N steps reference points into acados
        void solve();                                                // solve MPC with EKF
        
        MatrixXd RK4(MatrixXd x, MatrixXd u);                   // EKF predict and update
        MatrixXd f(MatrixXd x, MatrixXd u);                     // system process model
        MatrixXd h(MatrixXd x);                                 // measurement model
        MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);    // compute Jacobian of system process model
        MatrixXd compute_jacobian_H(MatrixXd x);                // compute Jacobian of measurement model

};

#endif