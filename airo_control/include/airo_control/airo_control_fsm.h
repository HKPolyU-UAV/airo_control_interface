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
#include "airo_control/disturbance_observer.h"
#include "airo_control/controller/mpc.h"
#include "airo_control/controller/backstepping.h"
#include "airo_control/controller/slidingmode.h"
#include <gazebo_msgs/ApplyBodyWrench.h>



class AIRO_CONTROL_FSM{
    private:

    enum STATE_FSM{
		RC_MANUAL,
		AUTO_HOVER,
		AUTO_TAKEOFF,
		AUTO_LAND,
		POS_COMMAND
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
	bool WITHOUT_RC,ENABLE_OBSERVER,APPLY_OBSERVER;

	// Variables
	STATE_FSM state_fsm;
	RC_INPUT rc_input;
	RC_INPUT::RC_PARAM rc_param;
	bool solve_controller;
	bool is_landed;
	bool is_armed;
	bool enable_preview = false; // Only for MPC
	bool use_preview = false; // Only for MPC

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
	ros::Publisher disturbance_pub;

	// ROS Services
	ros::ServiceClient setmode_srv;
	ros::ServiceClient arm_srv;
	ros::ServiceClient reboot_srv;
	ros::ServiceClient body_wrench_client;
	
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
	geometry_msgs::TwistStamped local_twist;
	geometry_msgs::AccelStamped local_accel;
	mavros_msgs::AttitudeTarget attitude_target;
	mavros_msgs::State current_state;
	mavros_msgs::State previous_state;
	mavros_msgs::ExtendedState current_extended_state;

	// Controller & Observer
	std::unique_ptr<BASE_CONTROLLER> controller;
	std::unique_ptr<DISTURBANCE_OBSERVER> disturbance_observer;

	// Disturbance
	geometry_msgs::Vector3Stamped force_disturbance;

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
	void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
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
};

#endif