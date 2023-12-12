#include "airo_control/airo_control_fsm_ekf.h"

AIRO_CONTROL_FSM::AIRO_CONTROL_FSM(ros::NodeHandle& nh){
    // ROS Parameters
    nh.getParam("airo_control_node/fsm/controller_type",CONTROLLER_TYPE);
    nh.getParam("airo_control_node/fsm/pose_topic",POSE_TOPIC);
    nh.getParam("airo_control_node/fsm/twist_topic",TWIST_TOPIC);
    nh.getParam("airo_control_node/fsm/state_timeout",STATE_TIMEOUT);
    nh.getParam("airo_control_node/fsm/rc_timeout",RC_TIMEOUT);
    nh.getParam("airo_control_node/fsm/odom_timeout",ODOM_TIMEOUT);
    nh.getParam("airo_control_node/fsm/command_timeout",COMMAND_TIMEOUT);
    nh.getParam("airo_control_node/fsm/motor_speedup_time",MOTOR_SPEEDUP_TIME);
    nh.getParam("airo_control_node/fsm/takeoff_height",TAKEOFF_HEIGHT);
    nh.getParam("airo_control_node/fsm/takeoff_land_speed",TAKEOFF_LAND_SPEED);
    nh.getParam("airo_control_node/fsm/reject_takeoff_twist_threshold",REJECT_TAKEOFF_TWIST_THRESHOLD);
    nh.getParam("airo_control_node/fsm/hover_max_velocity",HOVER_MAX_VELOCITY);
    nh.getParam("airo_control_node/fsm/hover_max_yaw_rate",HOVER_MAX_YAW_RATE);
    nh.getParam("airo_control_node/fsm/check_safety_volumn",CHECK_SAFETY_VOLUMN);
    nh.getParam("airo_control_node/fsm/safety_volumn",SAFETY_VOLUMN); // min_x max_x min_y max_y max_z
    nh.getParam("airo_control_node/fsm/without_rc",WITHOUT_RC);

    nh.getParam("airo_control_node/fsm/throttle_channel",rc_param.THROTTLE_CHANNEL);
    nh.getParam("airo_control_node/fsm/yaw_channel",rc_param.YAW_CHANNEL);
    nh.getParam("airo_control_node/fsm/pitch_channel",rc_param.PITCH_CHANNEL);
    nh.getParam("airo_control_node/fsm/roll_channel",rc_param.ROLL_CHANNEL);
    nh.getParam("airo_control_node/fsm/fsm_channel",rc_param.FSM_CHANNEL);
    nh.getParam("airo_control_node/fsm/command_channel",rc_param.COMMAND_CHANNEL);
    nh.getParam("airo_control_node/fsm/reboot_channel",rc_param.REBOOT_CHANNEL);
    nh.getParam("airo_control_node/fsm/kill_channel",rc_param.KILL_CHANNEL);
    nh.getParam("airo_control_node/fsm/reverse_throttle",rc_param.REVERSE_THROTTLE);
    nh.getParam("airo_control_node/fsm/reverse_yaw",rc_param.REVERSE_YAW);
    nh.getParam("airo_control_node/fsm/reverse_pitch",rc_param.REVERSE_PITCH);
    nh.getParam("airo_control_node/fsm/reverse_roll",rc_param.REVERSE_ROLL);
    nh.getParam("airo_control_node/fsm/switch_threshold",rc_param.SWITCH_THRESHOLD);
    nh.getParam("airo_control_node/fsm/joystick_deadzone",rc_param.JOYSTICK_DEADZONE);
    nh.getParam("airo_control_node/fsm/check_centered_threshold",rc_param.CHECK_CENTERED_THRESHOLD);

    // Initialize EKF
    
    // Initialize body wrench force
    applied_wrench.fx = 10.0;
    applied_wrench.fy = 0.0;
    applied_wrench.fz = 0.0;
    applied_wrench.tx = 0.0;
    applied_wrench.ty = 0.0;
    applied_wrench.tz = 0.0;

    // ROS Sub & Pub
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC,5,&AIRO_CONTROL_FSM::pose_cb,this);
    twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(TWIST_TOPIC,5,&AIRO_CONTROL_FSM::twist_cb,this);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",5,&AIRO_CONTROL_FSM::imu_cb,this);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",1,&AIRO_CONTROL_FSM::state_cb,this);
    extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",1,&AIRO_CONTROL_FSM::extended_state_cb,this);
    rc_input_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",1,&AIRO_CONTROL_FSM::rc_input_cb,this);
    command_sub = nh.subscribe<airo_message::Reference>("/airo_control/setpoint",1,&AIRO_CONTROL_FSM::external_command_cb,this);
    command_preview_sub = nh.subscribe<airo_message::ReferencePreview>("/airo_control/setpoint_preview",5,&AIRO_CONTROL_FSM::external_command_preview_cb,this);
    takeoff_land_sub = nh.subscribe<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",1,&AIRO_CONTROL_FSM::takeoff_land_cb,this);
    setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",1);
    fsm_info_pub = nh.advertise<airo_message::FSMInfo>("/airo_control/fsm_info",1);
    esti_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/airco_control/ekf/pose",5);
    esti_disturbance_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_control/ekf/disturbance",5);
    applied_disturbance_pub = nh.advertise<geometry_msgs::PoseStamped>("/airo_control/applied_disturbance",5);

    // ROS Services
    setmode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    reboot_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    wrench_srv = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    // Initialize FSM & Controller 
    state_fsm = RC_MANUAL;
    if (CONTROLLER_TYPE == "mpc"){
        controller = std::make_unique<MPC>(nh);
    }
    else if (CONTROLLER_TYPE == "backstepping"){
        controller = std::make_unique<BACKSTEPPING>(nh);
    }
    else if (CONTROLLER_TYPE == "slidingmode"){
        controller = std::make_unique<SLIDINGMODE>(nh);
    }
    else {
        ROS_ERROR("[AIRo Control] Invalid controller type!");
    }
    rc_input.set_rc_param(rc_param);
    controller_ref.header.stamp = ros::Time::now();

    // For MPC Preview
    if(auto dummy_mpc = dynamic_cast<MPC*>(controller.get())){
        enable_preview = dummy_mpc->param.enable_preview;
        if (enable_preview){
            controller_ref_preview.header.stamp = ros::Time::now();
            controller_ref_preview.ref_pose.resize(QUADROTOR_N+1);
            controller_ref_preview.ref_twist.resize(QUADROTOR_N+1);
            controller_ref_preview.ref_accel.resize(QUADROTOR_N+1);
        }
    }

    // Wait for vehicle connection
    while(!current_state.connected || !odom_received(ros::Time::now())){
        if (!current_state.connected){
            ROS_WARN_STREAM_THROTTLE(5.0,"[AIRo Control] Not yet connected to vehicle!");
        }
        else {
            ROS_WARN_STREAM_THROTTLE(5.0,"[AIRo Control] Odom not received!");
        }
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    ROS_INFO("[AIRo Control] Connected to vehicle. FSM spinning!");
}

void AIRO_CONTROL_FSM::process(){
    // Step 1: Update varialbes
    current_time = ros::Time::now();
    solve_controller = false;
    fsm_info.is_waiting_for_command = false;

    // Step 2: State machine
    if (check_connection(current_time)){
        fsm();
    }
    else{
        return;
    }

    // Step 3: Solve position controller if needed
    if(solve_controller){
        if (!use_preview){
            attitude_target = controller->solve(local_pose,local_twist,local_accel,controller_ref);
        }
        else{
            MPC* dummy_mpc = dynamic_cast<MPC*>(controller.get());
            attitude_target = dummy_mpc->solve(local_pose,local_twist,local_accel,controller_ref_preview);
        }
    }

    // Step 4: Detect if landed
    land_detector();

    // Step 5: Publish control commands and fsm state
    if (state_fsm != RC_MANUAL){
        publish_control_commands(attitude_target,current_time);
    }
    fsm_info.header.stamp = current_time;
    fsm_info.is_landed = is_landed;
    fsm_info_pub.publish(fsm_info);

    // Step 6: Reset all variables
	rc_input.enter_fsm = false;
	rc_input.enter_reboot = false;
    use_preview = false;
}

void AIRO_CONTROL_FSM::fsm(){
    switch (state_fsm){
        case RC_MANUAL:{
            // Update is_landed according to external state
            is_landed = !current_state.armed;

            // To AUTO_HOVER
            if (rc_input.enter_fsm && !is_landed){
                if (!odom_received(current_time)){
                    ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] Reject AUTO_HOVER. No odom or imu!");
                    break;
                }
                if (toggle_offboard(true)){
                    auto_hover_init();
                    state_fsm = AUTO_HOVER;
                    ROS_INFO("\033[32m[AIRo Control] RC_MANUAL ==>> AUTO_HOVER\033[32m");
                }
                break;
            }

            // To AUTO_TAKEOFF
            else if ((rc_input.enter_fsm && !rc_input.is_command && is_landed)
                  || (takeoff_trigered(current_time) && rc_input.is_command && is_landed)){
                if (!odom_received(current_time)){
                    ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] Reject AUTO_TAKEOFF. No odom or imu!");
                    break;
                }
                if (twist_norm(local_twist) > REJECT_TAKEOFF_TWIST_THRESHOLD){
                    ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] Reject AUTO_TAKEOFF. Norm Twist=" << twist_norm(local_twist) << "m/s, dynamic takeoff is not allowed!");
                    break;
                }
                if (!rc_received(current_time)){
                    if (WITHOUT_RC){
                        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Takeoff without RC. Take extra caution!");
                    }
                    else{
                        ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] RC not connected. Reject AUTO_TAKEOFF!");
                        break;
                    }
                }
                if ((!rc_input.is_fsm || !rc_input.check_centered()) && rc_received(current_time)){
                    ROS_ERROR("[AIRo Control] Reject AUTO_TAKEOFF. Center the joysticks and enable offboard switch!");
                    while (ros::ok()){
                        ros::Duration(0.1).sleep();
                        ros::spinOnce();
                        if (rc_input.is_fsm && rc_input.check_centered()){
                            current_time = ros::Time::now();
                            break;
                        }
                    }
                }
			    if (toggle_offboard(true)){
                    if(toggle_arm(true)){
                        // Wait for several seconds to warn prople
                        takeoff_land_init();
                        get_motor_speedup();
                        if (current_state.armed){
                            state_fsm = AUTO_TAKEOFF;
                            ROS_INFO("\033[32m[AIRo Control] RC_MANUAL ==>> AUTO_TAKEOFF\033[32m");
                            break;
                        }
                        else{
                            ROS_ERROR("[AIRo Control] Takeoff failed! Vehicle cannot arm!");
                        }
                    }
                }
                break;
            }

            // Try to reboot
            else if (rc_input.enter_reboot && is_landed){
                if (current_state.armed){
                    ROS_ERROR("[AIRo Control] Reject reboot! Disarm the vehicle first!");
                    break;
                }
                reboot();
            }

            else{
                ROS_INFO_STREAM_THROTTLE(5.0,"[AIRo Control] Waiting for commands in RC_MANUAL mode!");
            }

            break;
        }

        case AUTO_TAKEOFF:{
            // To RC_MANUAL
            if (!rc_input.is_fsm || !odom_received(current_time) || !current_state.armed){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_fsm){
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
                else if(!odom_received(current_time)){
                ROS_ERROR("[AIRo Control] No odom or imu! Switching to RC_MANUAL mode.");
                }
                else{
                ROS_WARN("[AIRo Control] Vehicle disarmed!");
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
            }
            else{
                // Reach the desired height
                if (local_pose.pose.position.z > (takeoff_land_pose.pose.position.z + TAKEOFF_HEIGHT)){
                    auto_hover_init();
                    state_fsm = AUTO_HOVER;
                    ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> AUTO_HOVER\033[32m");
                }
                // Send takeoff reference
                else{
                    set_takeoff_land_ref(TAKEOFF_LAND_SPEED);
                }
            }

            break;
        }

        case AUTO_HOVER:{
            // To RC_MANUAL
            if (!rc_input.is_fsm || !odom_received(current_time) || !current_state.armed){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_fsm){
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
                else if(!odom_received(current_time)){
                ROS_ERROR("[AIRo Control] No odom or imu! Switching to RC_MANUAL mode.");
                }
                else{
                ROS_WARN("[AIRo Control] Vehicle disarmed!");
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
            }

            // To AUTO_LAND
            else if (land_trigered(current_time) && rc_input.is_command){
                if (external_command_received(current_time)){
                    ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Reject AUTO_LAND mode. Stop sending external commands and try again!");
                }
                else{
                    takeoff_land_init();
                    state_fsm = AUTO_LAND;
                    ROS_INFO("\033[32m[AIRo Control] AUTO_HOVER ==>> AUTO_LAND\033[32m");
                }
            }

            // To POS_COMMAND
            else if ((external_command_preview_received(current_time) && rc_input.is_command && enable_preview) ||
                    (external_command_received(current_time) && rc_input.is_command)){
                state_fsm = POS_COMMAND;
                ROS_INFO("\033[32m[AIRo Control] AUTO_HOVER ==>> POS_COMMAND\033[32m");
            }

            // Disarm
            else if (is_landed){
                motor_idle_and_disarm();
                ROS_INFO("\033[32m[AIRo Control] AUTO_HOVER ==>> RC_MANUAL\033[32m");
                break;
            }

            // AUTO_HOVER
            set_ref_with_rc();
            if (rc_input.is_command){
                fsm_info.is_waiting_for_command = true;
            }

            break;
        }

        case AUTO_LAND:{
            // To RC_MANUAL
            if (!rc_input.is_fsm || !odom_received(current_time) || !current_state.armed){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_fsm){
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
                else if(!odom_received(current_time)){
                ROS_ERROR("[AIRo Control] No odom or imu! Switching to RC_MANUAL mode.");
                }
                else{
                ROS_WARN("[AIRo Control] Vehicle disarmed!");
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
            }

            // To AUTO_HOVER
            else if (!rc_input.is_command){
                auto_hover_init();
                state_fsm = AUTO_HOVER;
                ROS_INFO("\033[32m[AIRo Control] AUTO_LAND ==>> AUTO_HOVER\033[32m");
            }

            // Send land reference
            else if (!is_landed){
                set_takeoff_land_ref(-TAKEOFF_LAND_SPEED);
            }
            // Disarm vehicle
            else{
                motor_idle_and_disarm();
                ROS_INFO("\033[32m[AIRo Control] AUTO_LAND ==>> RC_MANUAL\033[32m");
            }

            break;            
        }

        case POS_COMMAND:{
            // To RC_MANUAL
            if (!rc_input.is_fsm || !odom_received(current_time) || !current_state.armed){
                state_fsm = RC_MANUAL;
                toggle_offboard(false);
                if(!rc_input.is_fsm){
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
                else if(!odom_received(current_time)){
                ROS_ERROR("[AIRo Control] No odom or imu! Switching to RC_MANUAL mode.");
                }
                else{
                ROS_WARN("[AIRo Control] Vehicle disarmed!");
                ROS_INFO("\033[32m[AIRo Control] AUTO_TAKEOFF ==>> RC_MANUAL\033[32m");
                }
            }

            // To AUTO_HOVER
            else if (!rc_input.is_command || 
            (!external_command_received(current_time) && !external_command_preview_received(current_time)) ||
            (external_command_preview_received(current_time) && !enable_preview)){
                auto_hover_init();
                state_fsm = AUTO_HOVER;
                ROS_INFO("\033[32m[AIRo Control] POS_COMMAND ==>> AUTO_HOVER\033[32m");  
            }

            // Follow command
            else{
                if(external_command_preview_received(current_time)){
                    set_ref_with_external_command_preview();
                    use_preview = true;
                }
                else{
                    set_ref_with_external_command();
                }
            }
            break;
        }
    }
}

bool AIRO_CONTROL_FSM::check_connection(const ros::Time& time){
    // Connected to vehicle
    bool is_connected = state_received(time) && current_state.connected;
    if (!is_connected){
        ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Connection to FCU lost! FSM reset to RC_MANUAL mode!");
        state_fsm = RC_MANUAL;
    }

    // Kill switch engaged
    bool is_killed = rc_received(time) && rc_input.is_killed;
    if (is_killed){
        ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Kill switch engaged. FSM reset to RC_MANUAL mode!");
        state_fsm = RC_MANUAL;
    }

    return is_connected && !is_killed;
}

void AIRO_CONTROL_FSM::publish_control_commands(mavros_msgs::AttitudeTarget target,ros::Time time){
    attitude_target.header.stamp = time;
    attitude_target.header.frame_id = std::string("FCU");
    attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    setpoint_pub.publish(attitude_target);
}

bool AIRO_CONTROL_FSM::toggle_offboard(bool flag){
	mavros_msgs::SetMode offboard_setmode;
    mavros_msgs::AttitudeTarget dummy_target;
    dummy_target.thrust = 0.0;
    dummy_target.orientation.w = 1.0;
    dummy_target.orientation.x = 0.0;
    dummy_target.orientation.y = 0.0;
    dummy_target.orientation.z = 0.0;

	if (flag){
		previous_state = current_state;
		if (previous_state.mode == "OFFBOARD"){
            previous_state.mode = "MANUAL"; // Not allowed
        }
		offboard_setmode.request.custom_mode = "OFFBOARD";

        // Start by streaming setpoints
        for(int i = 10; ros::ok() && i > 0; --i){
            setpoint_pub.publish(attitude_target);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        ros::Time offboard_start;
        while(ros::ok() && (ros::Time::now() - offboard_start).toSec() > 2.0){
            if (setmode_srv.call(offboard_setmode) && offboard_setmode.response.mode_sent){
                ROS_INFO_STREAM_THROTTLE(1.0,"[AIRo Control] Offboard mode enabled!");
                return true;
            }
            setpoint_pub.publish(attitude_target);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Can not enable offboard mode!");
        return false;
	}
	else{
		offboard_setmode.request.custom_mode = previous_state.mode;
		if (!(setmode_srv.call(offboard_setmode) && offboard_setmode.response.mode_sent)){
			ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Exit OFFBOARD rejected by PX4!");
			return false;
		}
        else{
            ROS_WARN("[AIRo Control] Exiting OFFBOARD mode!");
        }
	}
	return true;
}

bool AIRO_CONTROL_FSM::toggle_arm(bool flag){
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = flag;

	if (!(arm_srv.call(arm_cmd) && arm_cmd.response.success)){
		if (flag)
			ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Arm rejected by PX4!");
		else
			ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Disarm rejected by PX4!");

		return false;
	}

    if (flag){
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Vehicle arming!");
    }
    else{
        ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Vehicle disarmed!");
    }
    
	return true; 
}

void AIRO_CONTROL_FSM::get_motor_speedup(){
    while(ros::ok() && (current_time - takeoff_land_time).toSec() < MOTOR_SPEEDUP_TIME){
        double delta_t = (current_time - takeoff_land_time).toSec();
	    double ref_thrust = (delta_t/MOTOR_SPEEDUP_TIME)*controller->get_hover_thrust()*0.65 + 0.005;

        attitude_target.thrust = ref_thrust;
        attitude_target.orientation.w = takeoff_land_pose.pose.orientation.w;
        attitude_target.orientation.x = takeoff_land_pose.pose.orientation.x;
        attitude_target.orientation.y = takeoff_land_pose.pose.orientation.y;
        attitude_target.orientation.z = takeoff_land_pose.pose.orientation.z;

        publish_control_commands(attitude_target,current_time);
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
    }
}

void AIRO_CONTROL_FSM::set_ref(const geometry_msgs::PoseStamped& pose){
    controller_ref.header.stamp = pose.header.stamp;
    geometry_msgs::Pose dummy_pose;
    dummy_pose.position = pose.pose.position;
    dummy_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, extract_yaw_from_quaternion(pose.pose.orientation));
    
    controller_ref.ref_pose = dummy_pose;
    controller_ref.ref_twist.linear.x = 0;
    controller_ref.ref_twist.linear.y = 0;
    controller_ref.ref_twist.linear.z = 0;
    controller_ref.ref_accel.linear.x = 0;
    controller_ref.ref_accel.linear.y = 0;
    controller_ref.ref_accel.linear.z = 0;

    solve_controller = true;
}

void AIRO_CONTROL_FSM::set_takeoff_land_ref(const double speed){
    current_time = ros::Time::now();
    double delta_t = (current_time - takeoff_land_time).toSec() - (speed > 0 ? MOTOR_SPEEDUP_TIME : 0);
    geometry_msgs::PoseStamped takeoff_land_ref;
    takeoff_land_ref.header.stamp = current_time;
    takeoff_land_ref.pose.position.x = takeoff_land_pose.pose.position.x;
    takeoff_land_ref.pose.position.y = takeoff_land_pose.pose.position.y;
    takeoff_land_ref.pose.position.z = takeoff_land_pose.pose.position.z + speed * delta_t;
    takeoff_land_ref.pose.position = check_safety_volumn(takeoff_land_ref.pose.position);
    takeoff_land_ref.pose.orientation = takeoff_land_pose.pose.orientation;

	set_ref(takeoff_land_ref);
}

void AIRO_CONTROL_FSM::set_ref_with_rc(){
    current_time = ros::Time::now();
    double delta_t = (current_time - last_hover_time).toSec();
    geometry_msgs::PoseStamped rc_ref;
    double rc_psi, controller_psi;

    controller_psi = extract_yaw_from_quaternion(controller_ref.ref_pose.orientation);
    rc_ref.header.stamp = current_time;
    rc_ref.pose.position.x = controller_ref.ref_pose.position.x + rc_input.channel[rc_param.PITCH_CHANNEL-1]*HOVER_MAX_VELOCITY*delta_t*(rc_param.REVERSE_PITCH ? -1 : 1);
    rc_ref.pose.position.y = controller_ref.ref_pose.position.y - rc_input.channel[rc_param.ROLL_CHANNEL-1]*HOVER_MAX_VELOCITY*delta_t*(rc_param.REVERSE_ROLL ? -1 : 1);
    rc_ref.pose.position.z = controller_ref.ref_pose.position.z + rc_input.channel[rc_param.THROTTLE_CHANNEL-1]*HOVER_MAX_VELOCITY*delta_t*(rc_param.REVERSE_THROTTLE ? -1 : 1);
    rc_psi = controller_psi - rc_input.channel[rc_param.YAW_CHANNEL-1]*HOVER_MAX_YAW_RATE*delta_t*(rc_param.REVERSE_YAW ? -1 : 1);

    rc_ref.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, rc_psi);
    rc_ref.pose.position = check_safety_volumn(rc_ref.pose.position);
    set_ref(rc_ref);
    last_hover_time = current_time;
}

void AIRO_CONTROL_FSM::set_ref_with_external_command(){
    controller_ref.header = external_command.header;
    controller_ref.ref_pose.position = check_safety_volumn(external_command.ref_pose.position);
    controller_ref.ref_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, extract_yaw_from_quaternion(external_command.ref_pose.orientation));
    controller_ref.ref_twist = external_command.ref_twist;
    controller_ref.ref_accel = external_command.ref_accel;

    solve_controller = true;
    fsm_info.is_waiting_for_command = true;
}

void AIRO_CONTROL_FSM::set_ref_with_external_command_preview(){
    controller_ref_preview.header = external_command_preview.header;
    for(int i = 0; i < QUADROTOR_N+1; i++){
        controller_ref_preview.ref_pose[i].position = check_safety_volumn(external_command_preview.ref_pose[i].position);
        controller_ref_preview.ref_pose[i].orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, extract_yaw_from_quaternion(external_command_preview.ref_pose[i].orientation));
    }
    controller_ref_preview.ref_twist = external_command_preview.ref_twist;
    controller_ref_preview.ref_accel = external_command_preview.ref_accel;

    solve_controller = true;
    fsm_info.is_waiting_for_command = true;
}

double AIRO_CONTROL_FSM::extract_yaw_from_quaternion(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    double phi,theta,psi;
    tf::Matrix3x3(tf_quaternion).getRPY(phi, theta, psi);
    return psi;
}

void AIRO_CONTROL_FSM::land_detector(){
	static ros::Time time_C12_reached;
	static bool is_last_C12_satisfy;

	if (is_landed){
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}

	if (state_fsm == STATE_FSM::AUTO_TAKEOFF || state_fsm == STATE_FSM::POS_COMMAND){
		is_landed = false;
	}
    if (state_fsm == STATE_FSM::RC_MANUAL || state_fsm == STATE_FSM::AUTO_TAKEOFF || state_fsm == STATE_FSM::POS_COMMAND){
        return; // No need of other decisions
    }

	// Land_detector parameters
	constexpr double POSITION_DEVIATION = -1.0; // Constraint 1: target position below real position for POSITION_DEVIATION meters.
    constexpr double VELOCITY_THRESHOLD = 0.25; // Constraint 2: velocity below VELOCITY_THRESHOLD m/s.
	constexpr double TIME_KEEP = 2.0; // Constraint 3: Constraint 1&2 satisfied for TIME_KEEP seconds.

	if (!is_landed){
		bool C12_satisfy = (controller_ref.ref_pose.position.z - local_pose.pose.position.z) < POSITION_DEVIATION && twist_norm(local_twist) < VELOCITY_THRESHOLD;
        if (C12_satisfy && !is_last_C12_satisfy){
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy){
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP){ //Constraint 3 reached
				is_landed = true;
			}
		}
		is_last_C12_satisfy = C12_satisfy;
	}
}

void AIRO_CONTROL_FSM::motor_idle_and_disarm(){
    double last_disarm_time = 0;
    while(ros::ok()){
        if (current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){ // CONTROL allows disarm after this
            if (current_time.toSec() - last_disarm_time > 1.0){
                if (toggle_arm(false)){ // Successful disarm
                    state_fsm = RC_MANUAL;
                    toggle_offboard(false); // Toggle off offboard after disarm
                    break;
                }
                last_disarm_time = current_time.toSec();
            }
        }
        else{            
            attitude_target.thrust = 0.03;
            attitude_target.orientation.w = local_pose.pose.orientation.w;
            attitude_target.orientation.x = local_pose.pose.orientation.x;
            attitude_target.orientation.y = local_pose.pose.orientation.y;
            attitude_target.orientation.z = local_pose.pose.orientation.z;
            publish_control_commands(attitude_target,current_time);
        }
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        current_time = ros::Time::now();
    }
}

void AIRO_CONTROL_FSM::takeoff_land_init(){
    takeoff_land_pose = local_pose;
    takeoff_land_time = current_time;
}

void AIRO_CONTROL_FSM::auto_hover_init(){
    last_hover_time = current_time;
    set_ref(local_pose);
}

geometry_msgs::Point AIRO_CONTROL_FSM::check_safety_volumn(const geometry_msgs::Point& ref_point){
    if (CHECK_SAFETY_VOLUMN){
        geometry_msgs::Point safe_point;

        if(ref_point.x < SAFETY_VOLUMN[0]){ // x_ref < x_min
            safe_point.x = SAFETY_VOLUMN[0];
            ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] X command too small!");
        }
        else if (ref_point.x > SAFETY_VOLUMN[1]){ // x_ref > x_max
            safe_point.x = SAFETY_VOLUMN[1];
            ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] X command too large!");
        }
        else safe_point.x = ref_point.x; // x_min < x_ref < x_max

        if(ref_point.y < SAFETY_VOLUMN[2]){ // y_ref < y_min
            safe_point.y = SAFETY_VOLUMN[2];
            ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Y command too small!");
        }
        else if (ref_point.y > SAFETY_VOLUMN[3]){ // y_ref > y_max
            safe_point.y = SAFETY_VOLUMN[3];
            ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Y command too large!");
        }
        else safe_point.y = ref_point.y; // y_min < y_ref < y_max

        if (ref_point.z > SAFETY_VOLUMN[4]){ // z_ref > z_max
            safe_point.z = SAFETY_VOLUMN[4];
            ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Z command too large!");
        }
        else safe_point.z = ref_point.z; // z_ref < z_max

        return safe_point;
    }
    else{
        return ref_point;
    }

}

void AIRO_CONTROL_FSM::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header.stamp = msg->header.stamp;
    local_pose.pose = msg->pose;
}

void AIRO_CONTROL_FSM::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.header.stamp = msg->header.stamp;
    local_twist.twist = msg->twist;
}

void AIRO_CONTROL_FSM::imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    local_accel.header = msg->header;
    local_accel.accel.linear = msg->linear_acceleration;
    local_accel.accel.angular = msg->angular_velocity;
}

void AIRO_CONTROL_FSM::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void AIRO_CONTROL_FSM::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    current_extended_state = *msg;
}

void AIRO_CONTROL_FSM::rc_input_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    if(!WITHOUT_RC){
        rc_input.process(msg);
    }
}

void AIRO_CONTROL_FSM::external_command_cb(const airo_message::Reference::ConstPtr& msg){
    external_command.header.stamp = msg->header.stamp;
    external_command.ref_pose = msg->ref_pose;
    external_command.ref_twist = msg->ref_twist;
    external_command.ref_accel = msg->ref_accel;
    if (msg->ref_pose.orientation.w == 0.0 && msg->ref_pose.orientation.x == 0.0 && msg->ref_pose.orientation.y == 0.0 && msg->ref_pose.orientation.z == 0.0){
        external_command.ref_pose.orientation.w = 1.0;
    }
}

void AIRO_CONTROL_FSM::external_command_preview_cb(const airo_message::ReferencePreview::ConstPtr& msg){
    if(msg->ref_pose.size() == QUADROTOR_N+1 && msg->ref_twist.size() == QUADROTOR_N+1 && msg->ref_accel.size() == QUADROTOR_N+1){
        external_command_preview.header = msg->header;
        external_command_preview.ref_pose = msg->ref_pose;
        external_command_preview.ref_twist = msg->ref_twist;
        external_command_preview.ref_accel = msg->ref_accel;
        for (int i = 0; i < msg->ref_pose.size(); i++){
            if (msg->ref_pose[i].orientation.w == 0.0 && msg->ref_pose[i].orientation.x == 0.0 && msg->ref_pose[i].orientation.y == 0.0 && msg->ref_pose[i].orientation.z == 0.0){
                external_command.ref_pose.orientation.w = 1.0;
            } 
        }
    }
    else{
        ROS_ERROR_STREAM_THROTTLE(1.0,"[AIRo Control] Reference preview size should be " << (QUADROTOR_N+1) <<"!");
    }
}

void AIRO_CONTROL_FSM::takeoff_land_cb(const airo_message::TakeoffLandTrigger::ConstPtr& msg){
    takeoff_land_trigger.header.stamp = msg->header.stamp;
    takeoff_land_trigger.takeoff_land_trigger = msg->takeoff_land_trigger;
}

bool AIRO_CONTROL_FSM::state_received(const ros::Time& time){
    bool have_state = (time - current_state.header.stamp).toSec() < STATE_TIMEOUT;
    if (!have_state){
        ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] No MAVROS state received!");
        ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] " << (time - current_state.header.stamp).toSec() << "s since last state message!");
    }
    return have_state;
}

bool AIRO_CONTROL_FSM::rc_received(const ros::Time& time){
    return (time - rc_input.stamp).toSec() < RC_TIMEOUT;
}

bool AIRO_CONTROL_FSM::odom_received(const ros::Time& time){
    bool have_odom = (time - local_pose.header.stamp).toSec() < ODOM_TIMEOUT && 
    (time - local_twist.header.stamp).toSec() < ODOM_TIMEOUT;
    // if (!have_odom){
    //     ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] No odom received!");
    //     ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] " << (time - local_pose.header.stamp).toSec() << "s since last pose message!");
    //     ROS_ERROR_STREAM_THROTTLE(1.0, "[AIRo Control] " << (time - local_twist.header.stamp).toSec() << "s since last twist message!");
    // }
    return have_odom;
}

bool AIRO_CONTROL_FSM::external_command_received(const ros::Time& time){
    return (time - external_command.header.stamp).toSec() < COMMAND_TIMEOUT;
}

bool AIRO_CONTROL_FSM::external_command_preview_received(const ros::Time& time){
    return (time - external_command_preview.header.stamp).toSec() < COMMAND_TIMEOUT;
}

bool AIRO_CONTROL_FSM::takeoff_land_received(const ros::Time& time){
    return (time - takeoff_land_trigger.header.stamp).toSec() < COMMAND_TIMEOUT;
}

bool AIRO_CONTROL_FSM::takeoff_trigered(const ros::Time& time){
    return takeoff_land_received(time) && takeoff_land_trigger.takeoff_land_trigger;
}

bool AIRO_CONTROL_FSM::land_trigered(const ros::Time& time){
    return takeoff_land_received(time) && !takeoff_land_trigger.takeoff_land_trigger;
}

double AIRO_CONTROL_FSM::twist_norm(const geometry_msgs::TwistStamped twist){
    return sqrt(twist.twist.linear.x*twist.twist.linear.x + twist.twist.linear.y*twist.twist.linear.y + twist.twist.linear.z*twist.twist.linear.z);
}

void AIRO_CONTROL_FSM::reboot(){
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot;
	reboot.request.broadcast = false;
	reboot.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot.request.param1 = 1;	  // Reboot autopilot
	reboot.request.param2 = 0;	  // Do nothing for onboard computer
	reboot.request.confirmation = true;

	reboot_srv.call(reboot);

	ROS_INFO("FCU Rebooted!");
}

void AIRO_CONTROL_FSM::applyDisturbance(){
    // Call ros service apply_body_wrench
    wrench.request.body_name = "iris::base_link";  // Check the UAV's name in Gazebo
    wrench.request.reference_frame = "world";
    wrench.request.reference_point.x = 0.0;
    wrench.request.reference_point.y = 0.0;
    wrench.request.reference_point.z = 0.0;
    wrench.request.wrench.force.x = applied_wrench.fx;
    wrench.request.wrench.force.y = applied_wrench.fy;
    wrench.request.wrench.force.z = applied_wrench.fz;
    wrench.request.wrench.torque.x = applied_wrench.tx;
    wrench.request.wrench.torque.y = applied_wrench.ty;
    wrench.request.wrench.torque.z = applied_wrench.tz;
    wrench.request.start_time = ros::Time::now();
    wrench.request.duration = ros::Duration(1000);  // Duration of the disturbance
    body_wrench_client.call(wrench);
}

// Quaternion to euler angle
AIRO_CONTROL_FSM::Euler AIRO_CONTROL_FSM::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion, tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// Euler angle to quaternion
geometry_msgs::Quaternion AIRO_CONTROL_FSM::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

void AIRO_CONTROL_FSM::EKF(){
    // Get input u and measurement y
    meas_u << current_t.t0, current_t.t1, current_t.t2, current_t.t3, current_t.t4, current_t.t5;
    Matrix<double,6,1> tau;
    tau = K*meas_u;
    meas_y << local_pos.x, local_pos.y, local_pos.z, local_euler.phi, local_euler.theta, local_euler.psi,
            v_linear_body[0], v_linear_body[1], v_linear_body[2], v_angular_body[0], v_angular_body[1], v_angular_body[2],
            tau(0),tau(1),tau(2),tau(3),tau(4),tau(5);

    // Define Jacobian matrices of system dynamics and measurement model
    Matrix<double,18,18> F;                             // Jacobian of system dynamics
    Matrix<double,18,18> H;                             // Jacobian of measurement model

    // Define Kalman gain matrix
    Matrix<double,18,18> Kal;

    // Define prediction and update steps
    Matrix<double,18,1> x_pred;                         // Predicted state
    Matrix<double,18,18> P_pred;                        // Predicted covariance
    Matrix<double,18,1> y_pred;                         // Predicted measurement
    Matrix<double,18,1> y_err;                          // Measurement error

    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, meas_u);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, meas_u);                       // predict state at time k+1|k
    // dx = f(esti_x, meas_u);                             // acceleration
    P_pred = F * esti_P * F.transpose() + noise_Q;      // predict covariance at time k+1|k

    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                     // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred);                                 // predict measurement at time k+1
    y_err = meas_y - y_pred;                            // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                      // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred * (MatrixXd::Identity(n, n) - Kal * H).transpose() + Kal*noise_R*Kal.transpose(); // correct covariance estimate

    // body frame disturbance to inertial frame
    wf_disturbance << (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
            (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13) + (-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
            (-sin(meas_y(4)))*esti_x(12) + (cos(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
            esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16) + cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
            (cos(meas_y(3)))*esti_x(16) + (sin(meas_y(3)))*esti_x(17),
            (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16) + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);
    
    // publish estimate pose
    tf2::Quaternion quat;
    quat.setRPY(esti_x(3), esti_x(4), esti_x(5));
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    esti_pose.pose.pose.position.x = esti_x(0);
    esti_pose.pose.pose.position.y = esti_x(1);
    esti_pose.pose.pose.position.z = esti_x(2);
    esti_pose.pose.pose.orientation.x = quat_msg.x;
    esti_pose.pose.pose.orientation.y = quat_msg.y;
    esti_pose.pose.pose.orientation.z = quat_msg.z;
    esti_pose.pose.pose.orientation.w = quat_msg.w;
    esti_pose.twist.twist.linear.x = esti_x(6);
    esti_pose.twist.twist.linear.y = esti_x(7);
    esti_pose.twist.twist.linear.z = esti_x(8);
    esti_pose.twist.twist.angular.x = esti_x(9);
    esti_pose.twist.twist.angular.y = esti_x(10);
    esti_pose.twist.twist.angular.z = esti_x(11);
    esti_pose.header.stamp = ros::Time::now();
    esti_pose.header.frame_id = "odom_frame";
    esti_pose.child_frame_id = "base_link";
    esti_pose_pub.publish(esti_pose);

    // publish estimate disturbance
    esti_disturbance.pose.pose.position.x = wf_disturbance(0);
    esti_disturbance.pose.pose.position.y = wf_disturbance(1);
    esti_disturbance.pose.pose.position.z = wf_disturbance(2);
    esti_disturbance.twist.twist.angular.x = wf_disturbance(3);
    esti_disturbance.twist.twist.angular.y = wf_disturbance(4);
    esti_disturbance.twist.twist.angular.z = wf_disturbance(5);
    esti_disturbance.header.stamp = ros::Time::now();
    esti_disturbance.header.frame_id = "odom_frame";
    esti_disturbance.child_frame_id = "base_link";
    esti_disturbance_pub.publish(esti_disturbance);

    // publish estimate disturbance
    applied_disturbance.pose.pose.position.x = applied_wrench.fx;
    applied_disturbance.pose.pose.position.y = applied_wrench.fy;
    applied_disturbance.pose.pose.position.z = applied_wrench.fz;
    applied_disturbance.twist.twist.angular.x = applied_wrench.tx;
    applied_disturbance.twist.twist.angular.y = applied_wrench.ty;
    applied_disturbance.twist.twist.angular.z = applied_wrench.tz;
    applied_disturbance.header.stamp = ros::Time::now();
    applied_disturbance.header.frame_id = "odom_frame";
    applied_disturbance.child_frame_id = "base_link";
    applied_disturbance_pub.publish(applied_disturbance);

    // print estimate disturbance
    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        // std::cout << "esti_x12:   " << esti_x(12) << "\t esti_x2:  " << esti_x(2) << std::endl;
        std::cout << "tau_x:  " << meas_y(12) << "  tau_y:  " << meas_y(13) << "  tau_z:  " << meas_y(14) << "  tau_psi:  " << meas_y(17) << std::endl;
        std::cout << "acc_x:  " << body_acc.x << "  acc_y:  " << body_acc.y << "  acc_z:  " << body_acc.z << std::endl;
        std::cout << "acc_phi:  " << body_acc.phi << "  acc_theta:  " << body_acc.theta << "  acc_psi:  " << body_acc.psi << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_z:    " << acados_in.yref[0][2] << "\tref_yaw:    " << yaw_ref << std::endl;
        std::cout << "pos_x: " << meas_y(0) << "  pos_y: " << meas_y(1) << "  pos_z: " << meas_y(2) << " phi: " << meas_y(3) << "  theta: " << meas_y(4) << "  psi: " << meas_y(5) <<std::endl;
        std::cout << "esti_x: " << esti_x(0) << "  esti_y: " << esti_x(1) << "  esti_z: " << esti_x(2) << " esti_phi: " << esti_x(3) << "  esti_theta: " << esti_x(4) << "  esti_psi: " << esti_x(5) <<std::endl;
        //std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_z:  " << error_pose.pose.pose.position.z << std::endl;
        std::cout << "applied force x:  " << applied_wrench.fx << "\tforce y:  " << applied_wrench.fy << "\tforce_z:  " << applied_wrench.fz << std::endl;
        std::cout << "applied torque x:  " << applied_wrench.tx << "\ttorque y:  " << applied_wrench.ty << "\ttorque_z:  " << applied_wrench.tz << std::endl;
        std::cout << "(body frame) disturbance x: " << esti_x(12) << "    disturbance y: " << esti_x(13) << "    disturbance z: " << esti_x(14) << std::endl;
        //std::cout << "(world frame) disturbance x: " << wf_disturbance(0) << "    disturbance y: " << wf_disturbance(1) << "    disturbance z: " << wf_disturbance(2) << std::endl;
        //std::cout << "(world frame) disturbance phi: " << wf_disturbance(3) << "    disturbance theta: " << wf_disturbance(4) << "    disturbance psi: " << wf_disturbance(5) << std::endl;
        //std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        //std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

// 4th order RK for integration
MatrixXd BLUEROV2_DOB::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,1> k1;
    Matrix<double,18,1> k2;
    Matrix<double,18,1> k3;
    Matrix<double,18,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

/ Define system dynamics function
MatrixXd BLUEROV2_DOB::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,18,1> xdot;

    KAu = K*u;
    xdot << (cos(x(5))*cos(x(4)))*x(6) + (-sin(x(5))*cos(x(3))+cos(x(5))*sin(x(4))*sin(x(3)))*x(7) + (sin(x(5))*sin(x(3))+cos(x(5))*cos(x(3))*sin(x(4)))*x(8),  //xdot
            (sin(x(5))*cos(x(4)))*x(6) + (cos(x(5))*cos(x(3))+sin(x(3))*sin(x(4))*sin(x(5)))*x(7) + (-cos(x(5))*sin(x(3))+sin(x(4))*sin(x(5))*cos(x(3)))*x(8),
            (-sin(x(4)))*x(6) + (cos(x(4))*sin(x(3)))*x(7) + (cos(x(4))*cos(x(3)))*x(8),
            x(9) + (sin(x(5))*sin(x(4))/cos(x(4)))*x(10) + cos(x(3))*sin(x(4))/cos(x(4))*x(11),
            (cos(x(3)))*x(10) + (sin(x(3)))*x(11),
            (sin(x(3))/cos(x(4)))*x(10) + (cos(x(3))/cos(x(4)))*x(11), 
            invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl(0,0)*x(6)),    // xddot: M^-1[tau+w-C-g-D]
            invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl(1,1)*x(7)),
            invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl(2,2)*x(8)),
            invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl(3,3)*x(9)),
            invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl(4,4)*x(10)),
            invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl(5,5)*x(11)),
            // invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl(0,0)*x(6)+added_mass[2]*x(2)*x(4)),    // xddot: M^-1[tau+w-C-g-D]
            // invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl(1,1)*x(7)-added_mass[2]*x(2)*x(3)-added_mass[0]*x(0)*x(5)),
            // invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl(2,2)*x(8)-added_mass[1]*x(1)*x(3)+added_mass[0]*x(0)*x(4)),
            // invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl(3,3)*x(9)-added_mass[2]*x(2)*x(1)+added_mass[1]*x(1)*x(2)-added_mass[5]*x(5)*x(4)+added_mass[4]*x(4)*x(5)),
            // invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl(4,4)*x(10)+added_mass[2]*x(2)*x(0)-added_mass[0]*x(0)*x(2)+added_mass[5]*x(5)*x(3)-added_mass[3]*x(3)*x(5)),
            // invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl(5,5)*x(11)-added_mass[1]*x(1)*x(0)+added_mass[0]*x(0)*x(1)-added_mass[4]*x(4)*x(3)+added_mass[3]*x(3)*x(4)),
            0,0,0,0,0,0;
            
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd BLUEROV2_DOB::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,18,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),
        x(6),x(7),x(8),x(9),x(10),x(11),
        M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl(0,0)*x(6),        
        M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl(1,1)*x(7),
        M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl(2,2)*x(8),
        M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl(3,3)*x(9),
        M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl(4,4)*x(10),
        M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl(5,5)*x(11);
        // M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl(0,0)*x(6)-added_mass[2]*x(2)*x(4),        
        // M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl(1,1)*x(7)+added_mass[2]*x(2)*x(3)+added_mass[0]*x(0)*x(5),
        // M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl(2,2)*x(8)+added_mass[1]*x(1)*x(3)-added_mass[0]*x(0)*x(4),
        // M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl(3,3)*x(9)+added_mass[2]*x(2)*x(1)-added_mass[1]*x(1)*x(2)+added_mass[5]*x(5)*x(4)-added_mass[4]*x(4)*x(5),
        // M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl(4,4)*x(10)-added_mass[2]*x(2)*x(0)+added_mass[0]*x(0)*x(2)-added_mass[5]*x(5)*x(3)+added_mass[3]*x(3)*x(5),
        // M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl(5,5)*x(11)+added_mass[1]*x(1)*x(0)-added_mass[0]*x(0)*x(1)+added_mass[4]*x(4)*x(3)-added_mass[3]*x(3)*x(4);

    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_DOB::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,18,18> F;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_DOB::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,18,18> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}