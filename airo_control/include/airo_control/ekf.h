#include <iostream>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <airo_message/FSMInfo.h>
#include <airo_message/Reference.h>
#include <airo_message/ReferencePreview.h>
#include <airo_message/TakeoffLandTrigger.h>
#include "airo_control/rc_input.h"
#include "airo_control/controller/mpc.h"
#include "airo_control/controller/backstepping.h"
#include "airo_control/controller/slidingmode.h"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <tf/tf.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>

using namesapce Eigen;

class EKF{
    private:

    enum SystemStates{
        x = 0, //position
        y = 1,
        z = 2,
        u = 4, //velocity
        v = 5,
        w = 6,
        phi = 7, //Euler anagle in roll axis
        theta = 8, //Euler angle in pitch axis
        psi = 9, //Euler angle in yaw axis
        p = 9, //angular velocities
        q = 10,
        r = 11,
    };

    enum ControlInputs{
        u1 = 0, //thrust along z direction
        u2 = 1, //rolling moments
        u3 = 2, //pitching moments
        u4 = 3, //yawing moments
    };

    struct SolverInput{
        double x0;
        double yref;
    };

    struct SolverOutput{
        double u0;
        double x1;
        double status, kkt_res, cpu_time;
    };

    struct Euler{
        double phi;
        double theta;
        double psi;
    };

    struct pos{
        double x;
        double y;
        double z;
        double u;
        double v;
        double w;
        double p;
        double q;
        double r;
    };

    struct acc{
        double x;
        double y;
        double z;
    };

    struct SolverParam{
        double disturbance_x;
        double disturbance_y;
        double disturbance_z;
        double disturbance_phi;
        double disturbance_theta;
        double disturbance_psi;
    };

    struct thrust{
        double t0;
        double t1;
        double t2;
        double t3;
    };

    struct wrench{
        double fx;
        double fy;
        double fz;
        double tx;
        double ty;
        double tz;
    };

    //ROS message variables

    //Acados variables

    //dynamics parameters
    double dt = 0.05;
    double mass = 0.9;
    double Ix ;
    double Iy;
    double Iz;
    double g = 9.81;

    //EKF parameters
    Matrix<double,6,1> wf_disturbance;          //world frame disturbance
    Matrix<double,6,1> meas_u;                  //inputs
    int n = 18;                                 //state dimension
    int m = 18;                                 //measurement dimension
    Matrix<double,18,1> meas_y                  //measurement vector
    MatrixXd P0 = MatrixXd::Identity(m, m);     //initial covariance

    //Acados parameters
    std::string WRENCH_FX;
    std::string WRENCH_FY;
    std::string WRENCH_FZ;
    std::string WRENCH_TZ;
    int READ_WRENCH;                  //0: periodic disturbance; 1: random disturbance; 2: read wrench from text

    //other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;
    int rand_counter = 0;
    int fx_counter = 0;
    double dis_time = 0;
    double periodic_counter = 0;
    double logger_time;

    // Times
	ros::Time current_time;
	
	// ROS Sub & Pub
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber command_sub;
	ros::Subscriber command_preview_sub;
	ros::Publisher setpoint_pub;

    public: 
    EKF(ROS::NodeHandle&);                                        //constructor
    Euler q2rpy(constant geometry_msgs::Quaternion&);             //quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const Euler&);                //euler angle to quaternion
    void ref_cb (int line_to_read);                               //fill N steps reference points into acados
    void pose_cb (const geometry_msgs::PoseStamped::ConstPtr&)    //get current position

    //disturbance observer functions
    void applyBodyWrench();
    //void imu_cb(const sensor_msgs::Imu::ConstPtr&);               //get current orientation
    void EKF();
    MatrixXd RK4(MatrixXd x, MatrixXd u);                         //EKF predict & update
    MatrixXd f(MatrixXd x, MatrixXd u);                           //system process model
    MatrixXd h(MatrixXd x);                                       //meansurement model
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);          //compute Jacobian of system process model
    MatrixXd compute_jacobian_H(MatrixXd x);                      //compute Jacobian of measurement model

};