#ifndef AIRO_TRAJECTORY_UTILS_H
#define AIRO_TRAJECTORY_UTILS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_control/FSMInfo.h>
#include <airo_control/TakeoffLandTrigger.h>
#include <airo_control/Reference.h>

namespace AIRO_TRAJECTORY_UTILS{
    bool target_reached(const geometry_msgs::Point&);

    geometry_msgs::Quaternion yaw_to_quaternion(double);

    int readDataFromFile(const char*, std::vector<std::vector<double>>&);

    void takeoff();
}

#endif