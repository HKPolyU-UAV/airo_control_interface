#ifndef SLIDINGMODE_H
#define SLIDINGMODE_H

#include <iostream>

#include "airo_control/controller/base_controller.h"

class SLIDINGMODE : public BASE_CONTROLLER{
    private:
        struct Param : public BASE_CONTROLLER::Param{
            double k_xe,k_xs,k_xt,k_ye,k_ys,k_yt,k_ze,k_zs,k_zt;
        };

        double g = 9.80665;
        double e_x,e_dx,s_x,e_y,e_dy,s_y,e_z,e_dz,s_z,u_x,u_y;
        ros::Publisher debug_pub;

    public:
        Param param;
        SLIDINGMODE(ros::NodeHandle&);
        void pub_debug();
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&, const geometry_msgs::Vector3Stamped&);
        double get_hover_thrust();
        int sign(double&);
};

#endif