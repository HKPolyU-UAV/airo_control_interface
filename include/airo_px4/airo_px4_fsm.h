#ifndef Airo_PX4_FSM_H
#define Airo_PX4_FSM_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <airo_px4/FSMInfo.h>
#include <airo_px4/TakeoffLandTrigger.h>

#include "airo_px4/rc_input.h"
#include "airo_px4/quadrotor_mpc.h"


class AIRO_PX4_FSM{
    private:

    enum STATE_FSM{
		RC_MANUAL,
		AUTO_HOVER,
		AUTO_TAKEOFF,
		AUTO_LAND,
		POS_COMMAND
	};

	// Parameters
	std::string POSE_TOPIC;
	std::string TWIST_TOPIC;
	double MESSAGE_TIMEOUT;
	double MOTOR_SPEEDUP_TIME;
	double TAKEOFF_HEIGHT;
	double TAKEOFF_LAND_SPEED;
	double REJECT_TAKEOFF_TWIST_THRESHOLD;
	double HOVER_MAX_VELOCITY;
	double HOVER_MAX_RATE;
	bool CHECK_SAFETY_VOLUMN;
	std::vector<double> SAFETY_VOLUMN; // min_x max_x min_y max_y min_z max_z
	bool WITHOUT_RC;

	// Variables
	STATE_FSM state_fsm;
	RC_INPUT rc_input;
	RC_INPUT::RC_PARAM rc_param;
	QUADROTOR_MPC::SolverParam solver_param;
	bool solve_controller;
	bool is_landed;
	bool is_armed;

	// Times
	ros::Time current_time;
	ros::Time takeoff_land_time;
	ros::Time last_hover_time;
	
	// ROS Sub & Pub
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	ros::Subscriber state_sub;
	ros::Subscriber extended_state_sub;
	ros::Subscriber rc_input_sub;
	ros::Subscriber command_sub;
	ros::Subscriber takeoff_land_sub;
	ros::Publisher setpoint_pub;
	ros::Publisher fsm_info_pub;

	// ROS Services
	ros::ServiceClient setmode_srv;
	ros::ServiceClient arm_srv;
	ros::ServiceClient reboot_srv;

	// Messages
	airo_px4::TakeoffLandTrigger takeoff_land_trigger; // 1 for takeoff 0 for landing
	airo_px4::FSMInfo fsm_info;
	airo_px4::Reference mpc_ref;
	airo_px4::Reference external_command;
	geometry_msgs::PoseStamped local_pose;
	geometry_msgs::PoseStamped takeoff_land_pose;
	geometry_msgs::PoseStamped ref_pose;
	geometry_msgs::TwistStamped local_twist;
	mavros_msgs::AttitudeTarget attitude_target;
	mavros_msgs::State current_state;
	mavros_msgs::State previous_state;
	mavros_msgs::ExtendedState current_extended_state;

	//Controller
	QUADROTOR_MPC controller;

	public:

	AIRO_PX4_FSM(ros::NodeHandle&);
	void process();
	void fsm();
	void publish_control_commands(mavros_msgs::AttitudeTarget,ros::Time);
	bool toggle_offboard(bool);
	bool toggle_arm(bool);
	void get_motor_speedup();
	void set_ref(const geometry_msgs::PoseStamped&);
	void set_takeoff_land_ref(const double);
	void set_ref_with_rc();
	void set_ref_with_external_command();
	void reference_init();
	double extract_yaw_from_quaternion(const geometry_msgs::Quaternion&);
	void land_detector();
	void motor_idle_and_disarm();
	void takeoff_land_init();
	void auto_hover_init();
	geometry_msgs::Point check_safety_volumn(const geometry_msgs::Point&);
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
	void state_cb(const mavros_msgs::State::ConstPtr&);
	void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr&);
	void rc_input_cb(const mavros_msgs::RCIn::ConstPtr&);
	void external_command_cb(const airo_px4::Reference::ConstPtr&);
	void takeoff_land_cb(const airo_px4::TakeoffLandTrigger::ConstPtr&);
	bool rc_received(const ros::Time&);
	bool odom_received(const ros::Time&);
	bool external_command_received(const ros::Time&);
	bool takeoff_land_received(const ros::Time&);
	bool takeoff_trigered(const ros::Time&);
	bool land_trigered(const ros::Time&);
	double twist_norm(const geometry_msgs::TwistStamped);
	void reboot();
};

#endif