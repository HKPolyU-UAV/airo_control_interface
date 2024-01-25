#include "airo_control/airo_control_fsm.h"

AIRO_CONTROL_FSM::AIRO_CONTROL_FSM(ros::NodeHandle& nh){
    // ROS Parameters
    nh.getParam("airo_control_node/fsm/controller_type",CONTROLLER_TYPE);
    nh.getParam("airo_control_node/fsm/enable_observer",ENABLE_OBSERVER);
    nh.getParam("airo_control_node/fsm/apply_observer",APPLY_OBSERVER);
    if (ENABLE_OBSERVER == false && APPLY_OBSERVER == true){
        ROS_ERROR("[AIRo Control] Cannot apply observer when it's disabled!");
        APPLY_OBSERVER = false;
    }
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
    disturbance_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/airo_control/force_disturbance",1);

    // ROS Services
    setmode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    reboot_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    // Initialize FSM, Controller, and Observer
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
    disturbance_observer = std::make_unique<DISTURBANCE_OBSERVER>(nh,controller->get_hover_thrust());
    force_disturbance.vector.x = 0;
    force_disturbance.vector.y = 0;
    force_disturbance.vector.z = 0;

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

    // Step 3: Run observer and solve position controller if needed
    if(solve_controller){
        if (ENABLE_OBSERVER){
            force_disturbance = disturbance_observer->observe(local_pose,local_twist,attitude_target,local_accel);
        }

        if (APPLY_OBSERVER){
            if (CONTROLLER_TYPE != "mpc"){
                ROS_WARN_STREAM_THROTTLE(1.0,"[AIRo Control] Observed disturbances will not be used for controllers other than MPC");
            }
            if (!use_preview){
                attitude_target = controller->solve(local_pose,local_twist,local_accel,controller_ref,force_disturbance);
            }
            else{
                MPC* dummy_mpc = dynamic_cast<MPC*>(controller.get());
                attitude_target = dummy_mpc->solve(local_pose,local_twist,local_accel,controller_ref_preview,force_disturbance);
            }
        }
        else{
            if (!use_preview){
                attitude_target = controller->solve(local_pose,local_twist,local_accel,controller_ref);
            }
            else{
                MPC* dummy_mpc = dynamic_cast<MPC*>(controller.get());
                attitude_target = dummy_mpc->solve(local_pose,local_twist,local_accel,controller_ref_preview);
            }
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
    
    if (ENABLE_OBSERVER){
        force_disturbance.header.stamp = current_time;
        disturbance_pub.publish(force_disturbance);
    }

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