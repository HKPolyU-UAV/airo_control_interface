#ifndef AIRO_CONTROL_FSM_H
#define AIRO_CONTROL_FSM_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
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

using namespace Eigen;

class AIRO_CONTROL_FSM{
    private:

    enum STATE_FSM{
		RC_MANUAL,
		AUTO_HOVER,
		AUTO_TAKEOFF,
		AUTO_LAND,
		POS_COMMAND
	};

	enum SYSTEM_STATES{
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
		r = 11
	};

	enum CONTROL_INPUTS{
		u1 = 0,
		u2 = 1,
		u3 = 2,
		u4 = 3
	};

	struct WRENCH{
        double fx;
        double fy;
        double fz;
    };

	struct EULER{
        double phi;
        double theta;
        double psi;
    };

	struct POS{
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

	struct ACC{
		double x;
		double y;
		double z;
		double phi;
		double theta;
		double psi;
	};

	struct SOLVER_PARAM{
		double disturbance_x;
		double disturbance_y;
		double disturbance_z;
	};

	// Parameters
	std::string CONTROLLER_TYPE;
	std::string POSE_TOPIC;
	std::string TWIST_TOPIC;
	double STATE_TIMEOUT,RC_TIMEOUT,ODOM_TIMEOUT,COMMAND_TIMEOUT;
	double MOTOR_SPEEDUP_TIME;
	double TAKEOFF_HEIGHT;
	double TAKEOFF_LAND_SPEED;
	double REJECT_TAKEOFF_TWIST_THRESHOLD;
	double HOVER_MAX_VELOCITY;
	double HOVER_MAX_YAW_RATE;
	bool CHECK_SAFETY_VOLUMN;
	std::vector<double> SAFETY_VOLUMN; // min_x max_x min_y max_y max_z
	bool WITHOUT_RC;


	// Variables
	STATE_FSM state_fsm;
	RC_INPUT rc_input;
	RC_INPUT::RC_PARAM rc_param;
	bool solve_controller;
	bool is_landed;
	bool is_armed;
	bool enable_preview = false; // Only for MPC
	bool use_preview = false; // Only for MPC
	EULER local_euler;
	POS local_pos;
	POS pre_body_pos;
	ACC body_acc;

	// Times
	ros::Time current_time;
	ros::Time takeoff_land_time;
	ros::Time last_hover_time;
	
	// ROS Sub & Pub
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	ros::Subscriber imu_sub;
	ros::Subscriber state_sub;
	ros::Subscriber extended_state_sub;
	ros::Subscriber rc_input_sub;
	ros::Subscriber command_sub;
	ros::Subscriber command_preview_sub;
	ros::Subscriber takeoff_land_sub;
	ros::Publisher setpoint_pub;
	ros::Publisher fsm_info_pub;

	ros::Publisher ref_pose_pub;
	ros::Publisher error_pose_pub;

	ros::Publisher esti_pose_pub;
	ros::Publisher esti_disturbance_pub;
	ros::Publisher applied_disturbance_pub;

	// ROS Services
	ros::ServiceClient setmode_srv;
	ros::ServiceClient arm_srv;
	ros::ServiceClient reboot_srv;
	ros::ServiceClient wrench_srv;
	
	// Messages
	airo_message::TakeoffLandTrigger takeoff_land_trigger; // 1 for takeoff 0 for landing
	airo_message::FSMInfo fsm_info;
	airo_message::Reference controller_ref;
	airo_message::Reference external_command;
	airo_message::ReferencePreview controller_ref_preview;
	airo_message::ReferencePreview external_command_preview;
	geometry_msgs::PoseStamped local_pose;
	geometry_msgs::PoseStamped takeoff_land_pose;
	geometry_msgs::PoseStamped ref_pose;
	geometry_msgs::PoseStamped esti_pose;
	geometry_msgs::PoseStamped esti_disturbance;
	geometry_msgs::PoseStamped applied_disturbance;
	geometry_msgs::TwistStamped local_twist;
	geometry_msgs::AccelStamped local_accel;
	mavros_msgs::AttitudeTarget attitude_target;
	mavros_msgs::State current_state;
	mavros_msgs::State previous_state;
	mavros_msgs::ExtendedState current_extended_state;
	
	// Applybodywrench
	WRENCH applied_wrench;	
	gazebo_msgs::ApplyBodyWrench body_wrench;

	// Controller
	std::unique_ptr<BASE_CONTROLLER> controller;

	// Acados variables

	// Dynamics parameters
	double dt = 0.05;
	double g = 9.81; 
	Matrix<double,1,6> M_values;
	Matrix<double,6,6> M;                // Mass matrix
	Matrix<double,6,6> invM;             // Inverse mass matrix
	Matrix<double,3,1> v_linear_body;    // Linear velocity in body frame
	Matrix<double,3,1> v_angular_body;   // Angular velocity in body frame
	Matrix<double,3,3> R_ib;             // Rotation matrix for linear from inertial to body frame
	Matrix<double,3,3> T_ib;             // Rotation matrix for angular from inertial to body frame

	// EKF parameters
	Matrix<double,6,1> wf_disturbance;          // World frame disturbance
	Matrix<double,4,1> meas_u;                  // Inputs
	int n = 15;                                 // State dimension
	int m = 15;                                 // Measurement dimension
	Matrix<double,15,1> meas_y;                 // Measurement vector
	MatrixXd P0 = MatrixXd::Identity(m,m);      // Initial covariance
	Matrix<double,15,1> esti_x;                 // Estimate states
	Matrix<double,15,15> esti_p;                // Estimate covariance
	Matrix<double,1,15> Q_cov;                  // Process noise value
	Matrix<double,15,15> noise_Q;               // Process noise matrix
	MatrixXd noise_R = MatrixXd::Identity(m,m)*(pow(dt,4)/4);      // Measurement noise matrix

	// Acados parameter
	std::string REF_TRAJ;
	std::string WRENCH_FX;
	std::string WRENCH_FY;
	std::string WRENCH_FZ;

	// Other variables
	tf::Quaternion tf_quaternion;
	int cout_counter = 0;
	int rand_counter = 0;
	int fx_counter = 0;
	double dis_time = 0;
	double amplitudeScalingFactor_X;
	double amplitudeScalingFactor_Y;
	double amplitudeScalingFactor_Z;
	double amplitudeScalingFactor_N;

	double logger_time;



	public:

	AIRO_CONTROL_FSM(ros::NodeHandle&);
	void process();
	void fsm();
	bool check_connection(const ros::Time&);
	void publish_control_commands(mavros_msgs::AttitudeTarget,ros::Time);
	bool toggle_offboard(bool);
	bool toggle_arm(bool);
	void get_motor_speedup();
	void set_ref(const geometry_msgs::PoseStamped&);
	void set_takeoff_land_ref(const double);
	void set_ref_with_rc();
	void set_ref_with_external_command();
	void set_ref_with_external_command_preview();
	double extract_yaw_from_quaternion(const geometry_msgs::Quaternion&);
	void land_detector();
	void motor_idle_and_disarm();
	void takeoff_land_init();
	void auto_hover_init();
	geometry_msgs::Point check_safety_volumn(const geometry_msgs::Point&);
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&); // get current position 
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
	void imu_cb(const sensor_msgs::Imu::ConstPtr&);
	void state_cb(const mavros_msgs::State::ConstPtr&);
	void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr&);
	void rc_input_cb(const mavros_msgs::RCIn::ConstPtr&);
	void external_command_cb(const airo_message::Reference::ConstPtr&);
	void external_command_preview_cb(const airo_message::ReferencePreview::ConstPtr&);
	void takeoff_land_cb(const airo_message::TakeoffLandTrigger::ConstPtr&);
	bool state_received(const ros::Time&);
	bool rc_received(const ros::Time&);
	bool odom_received(const ros::Time&);
	bool external_command_received(const ros::Time&);
	bool external_command_preview_received(const ros::Time&);
	bool takeoff_land_received(const ros::Time&);
	bool takeoff_trigered(const ros::Time&);
	bool land_trigered(const ros::Time&);
	double twist_norm(const geometry_msgs::TwistStamped);
	void reboot();

	EULER q2rpy(const geometry_msgs::Quaternion&);                  // quaternion to euler angle
    geometry_msgs::Quaternion rpy2q(const EULER&);                  // euler angle to quaternion
	void ref_cb(int line_to_read);                                  // fill N steps reference points into acados
     

	//disturbance observer functions
	void applyDisturbance();
	void EKF();
	MatrixXd RK4(MatrixXd x, MatrixXd u);                   // EKF predict and update
    MatrixXd f(MatrixXd x, MatrixXd u);                     // system process model
    MatrixXd h(MatrixXd x);                                 // measurement model
    MatrixXd compute_jacobian_F(MatrixXd x, MatrixXd u);    // compute Jacobian of system process model
    MatrixXd compute_jacobian_H(MatrixXd x);                // compute Jacobian of measurement model
};

#endif