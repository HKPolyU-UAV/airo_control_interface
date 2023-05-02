#include <ros/ros.h>
#include "mavros_msgs/RCIn.h"

class RC_INPUT{
    public:

    ros::Time stamp;
    double channel[4]; // Pitch, Roll, Yaw, and Thrust mapped from 1000~2000 to -1~1
    double offboard_switch; // Switch position mapped from 0~2000 to -1~1
    double last_offboard_switch;
    double command_switch; // Switch position mapped from 0~2000 to -1~1
    double last_command_switch;
    double reboot_switch; // Switch position mapped from 0~2000 to -1~1
    double last_reboot_switch;
    bool is_offboard;
    bool enter_offboard;
    bool is_command;
    bool enter_command;
    bool enter_reboot;
    bool init_indicator;

    static constexpr double COMMAND_THRESHOLD = 0.75;
    static constexpr double OFFBOARD_THRESHOLD = 0.75;
    static constexpr double REBOOT_THRESHOLD = 0.5;
    static constexpr double JOYSTICK_DEADZONE = 0.1;
    static constexpr int OFFBOARD_SWITCH_CHANNEL = 5; // Channel in QGC
    static constexpr int COMMAND_SWITCH_CHANNEL = 7; // Channel in QGC
    static constexpr int REBOOT_SWITCH_CHANNEL = 8; // Channel in QGC

    RC_INPUT();
    void check_validity();
    bool check_centered();
    void process(const mavros_msgs::RCIn::ConstPtr&);
};