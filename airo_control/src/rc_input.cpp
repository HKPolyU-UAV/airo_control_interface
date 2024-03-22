#include "airo_control/rc_input.h"

RC_INPUT::RC_INPUT(){
    stamp = ros::Time(0);

    last_fsm_switch = -1;
    last_reboot_switch = -1;

    // Important for no RC flight
    is_fsm = true;
    enter_fsm = false;
    is_command = true;
    enter_reboot = false;
    is_killed = false;

    for (int i = 0; i < 4; ++i){
        channel[i] = 0.0;
    }
}

void RC_INPUT::process(const mavros_msgs::RCIn::ConstPtr& msg){
    stamp = ros::Time::now();

    for (int i = 0; i < 4; ++i){
        channel[i] = ((double)msg->channels[i] - 1500.0) / 500.0;
        if (channel[i] > rc_param.JOYSTICK_DEADZONE)
            channel[i] = (channel[i] - rc_param.JOYSTICK_DEADZONE) / (1 - rc_param.JOYSTICK_DEADZONE);
        else if (channel[i] < -rc_param.JOYSTICK_DEADZONE)
            channel[i] = (channel[i] + rc_param.JOYSTICK_DEADZONE) / (1 - rc_param.JOYSTICK_DEADZONE);
        else
            channel[i] = 0;
    }

    fsm_switch = ((double)msg->channels[rc_param.FSM_CHANNEL - 1] - 1000.0) / 1000.0;
    command_switch = ((double)msg->channels[rc_param.COMMAND_CHANNEL - 1] - 1000.0) / 1000.0;
    reboot_switch = ((double)msg->channels[rc_param.REBOOT_CHANNEL - 1] - 1000.0) / 1000.0;
    kill_switch = ((double)msg->channels[rc_param.KILL_CHANNEL - 1] - 1000.0) / 1000.0;

    check_validity();

    // Init last indicators for first run
    if (!init_indicator){
        init_indicator = true;
        last_fsm_switch = fsm_switch;
        last_reboot_switch = reboot_switch;
    }

    // Set fsm
    if (last_fsm_switch < rc_param.SWITCH_THRESHOLD && fsm_switch > rc_param.SWITCH_THRESHOLD)
        enter_fsm = true;
    else
        enter_fsm = false;

    if (fsm_switch > rc_param.SWITCH_THRESHOLD)
        is_fsm = true;
    else
        is_fsm = false;

    // Set command
    if (command_switch > rc_param.SWITCH_THRESHOLD)
        is_command = true;
    else
        is_command = false;

    // Set reboot

    if (last_reboot_switch < rc_param.SWITCH_THRESHOLD && reboot_switch > rc_param.SWITCH_THRESHOLD)
        enter_reboot = true;
    else
        enter_reboot = false;

    // Set kill
    if (kill_switch > rc_param.SWITCH_THRESHOLD)
        is_killed = true;
    else
        is_killed = false;

    // Update last indicators
    last_fsm_switch = fsm_switch;
    last_reboot_switch = reboot_switch;
}

void RC_INPUT::check_validity(){
    if (fsm_switch >= -1.1 && fsm_switch <= 1.1 && command_switch >= -1.1 && command_switch <= 1.1 && reboot_switch >= -1.1 && reboot_switch <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC input validity check fail. fsm=%f, command=%f, reboot=%f", fsm_switch, command_switch, reboot_switch);
    }
}

bool RC_INPUT::check_centered(){
    bool centered = abs(channel[0]) < rc_param.CHECK_CENTERED_THRESHOLD && abs(channel[1]) < rc_param.CHECK_CENTERED_THRESHOLD && abs(channel[2]) < rc_param.CHECK_CENTERED_THRESHOLD && abs(channel[3]) < rc_param.CHECK_CENTERED_THRESHOLD;
    return centered;
}

void RC_INPUT::set_rc_param(RC_INPUT::RC_PARAM& param){
    rc_param = param;
}