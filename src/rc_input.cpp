#include "airo_px4/rc_input.h"

RC_INPUT::RC_INPUT(){
    stamp = ros::Time(0);

    last_offboard_switch = -1;
    last_command_switch = -1;
    last_reboot_switch = -1;

    // Important for no RC flight
    is_offboard = true;
    enter_offboard = false;
    is_command = true;
    enter_command = false;
    enter_reboot = false;

    for (int i = 0; i < 4; ++i){
        channel[i] = 0.0;
    }
}

void RC_INPUT::process(const mavros_msgs::RCIn::ConstPtr& msg){
    stamp = ros::Time::now();

    for (int i = 0; i < 4; ++i){
        channel[i] = ((double)msg->channels[i] - 1500.0) / 500.0;
        if (channel[i] > JOYSTICK_DEADZONE)
            channel[i] = (channel[i] - JOYSTICK_DEADZONE) / (1 - JOYSTICK_DEADZONE);
        else if (channel[i] < -JOYSTICK_DEADZONE)
            channel[i] = (channel[i] + JOYSTICK_DEADZONE) / (1 - JOYSTICK_DEADZONE);
        else
            channel[i] = 0;
    }

    offboard_switch = ((double)msg->channels[OFFBOARD_SWITCH_CHANNEL - 1] - 1000.0) / 1000.0;
    command_switch = ((double)msg->channels[COMMAND_SWITCH_CHANNEL - 1] - 1000.0) / 1000.0;
    reboot_switch = ((double)msg->channels[REBOOT_SWITCH_CHANNEL - 1] - 1000.0) / 1000.0;

    check_validity();

    // Init last indicators for first run
    if (!init_indicator){
        init_indicator = true;
        last_offboard_switch = offboard_switch;
        last_command_switch = command_switch;
        last_reboot_switch = reboot_switch;
    }

    // Set offboard
    if (last_offboard_switch < OFFBOARD_THRESHOLD && offboard_switch > OFFBOARD_THRESHOLD)
        enter_offboard = true;
    else
        enter_offboard = false;

    if (offboard_switch > OFFBOARD_THRESHOLD)
        is_offboard = true;
    else
        is_offboard = false;

    // Set command
    if (is_offboard){
        if (last_command_switch < COMMAND_THRESHOLD && command_switch > COMMAND_THRESHOLD)
            enter_command = true;
        else
            enter_command = false;

        if (command_switch > COMMAND_THRESHOLD)
            is_command = true;
        else
            is_command = false;
    }

    // Set reboot
    if (!is_offboard && !is_command){
        if (last_reboot_switch < REBOOT_THRESHOLD && reboot_switch > REBOOT_THRESHOLD)
            enter_reboot = true;
        else
            enter_reboot = false;
    }
    else
        enter_reboot = false;

    // Update last indicators
    last_offboard_switch = offboard_switch;
    last_command_switch = command_switch;
    last_reboot_switch = reboot_switch;
}

void RC_INPUT::check_validity(){
    if (offboard_switch >= -1.1 && offboard_switch <= 1.1 && command_switch >= -1.1 && command_switch <= 1.1 && reboot_switch >= -1.1 && reboot_switch <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC input validity check fail. offboard=%f, command=%f, reboot=%f", offboard_switch, command_switch, reboot_switch);
    }
}

bool RC_INPUT::check_centered(){
    bool centered = abs(channel[0]) < JOYSTICK_DEADZONE && abs(channel[1]) < JOYSTICK_DEADZONE && abs(channel[2]) < JOYSTICK_DEADZONE && abs(channel[3]) < JOYSTICK_DEADZONE;
    return centered;
}