#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <ros/ros.h>
#include "mavros_msgs/RCIn.h"

class RC_INPUT{
    public:

    struct RC_PARAM{
        int THROTTLE_CHANNEL;
        int YAW_CHANNEL;
        int PITCH_CHANNEL;
        int ROLL_CHANNEL;
        int FSM_CHANNEL;
        int COMMAND_CHANNEL;
        int REBOOT_CHANNEL;
        int KILL_CHANNEL;
        bool REVERSE_THROTTLE;
        bool REVERSE_YAW;
        bool REVERSE_PITCH;
        bool REVERSE_ROLL;
        double SWITCH_THRESHOLD;
        double JOYSTICK_DEADZONE;
        double CHECK_CENTERED_THRESHOLD;
    };

    ros::Time stamp;
    RC_PARAM rc_param;
    double channel[4]; // Pitch, Roll, Yaw, and Thrust mapped from 1000~2000 to -1~1
    double fsm_switch; // Switch position mapped from 0~2000 to -1~1
    double last_fsm_switch;
    double command_switch; // Switch position mapped from 0~2000 to -1~1
    double reboot_switch; // Switch position mapped from 0~2000 to -1~1
    double last_reboot_switch;
    double kill_switch;
    bool is_fsm;
    bool enter_fsm;
    bool is_command;
    bool enter_reboot;
    bool is_killed;
    bool init_indicator;

    RC_INPUT();
    void check_validity();
    bool check_centered();
    void set_rc_param(RC_INPUT::RC_PARAM&);
    void process(const mavros_msgs::RCIn::ConstPtr&);
};

#endif