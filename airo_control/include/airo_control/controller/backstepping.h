#ifndef BACKSTEPPING_H
#define BACKSTEPPING_H

#include <iostream>

#include "airo_control/controller/base_controller.h"

class BACKSTEPPING : public BASE_CONTROLLER{
    private:
        struct Param : public BASE_CONTROLLER::Param{
            double k_x1,k_x2,k_y1,k_y2,k_z1,k_z2;
        };

        double g = 9.80665;
        double e_x1,e_x2,e_y1,e_y2,e_z1,e_z2,u_x,u_y;
        ros::Publisher debug_pub;

    public:
        Param param;
        BACKSTEPPING(ros::NodeHandle&);
        void pub_debug();
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&, const geometry_msgs::Vector3Stamped&);
        double get_hover_thrust();
};

#endif