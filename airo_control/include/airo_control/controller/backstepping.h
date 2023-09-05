#ifndef BACKSTEPPING_H
#define BACKSTEPPING_H

#include <iostream>

#include "airo_control/controller/base_controller.h"

class BACKSTEPPING : public BASE_CONTROLLER{
    private:
        struct Param : public BASE_CONTROLLER::Param{
            double k_x1,k_x2,k_y1,k_y2,k_z1,k_z2;
        };

        enum SystemStates{
            x = 0,
            y = 1,
            z = 2,
            u = 3,
            v = 4,
            w = 5,
            phi = 6,
            theta = 7,
            psi = 8,
        };

        enum ControlInputs{
            thrust = 0,
            phi_cmd = 1,
            theta_cmd = 2,
            psi_cmd = 3,
        };

        int debug_counter;
        double g = 9.80665;
        double e_x1,e_x2,e_y1,e_y2,e_z1,e_z2,u_x,u_y;

    public:
        Param param;
        BACKSTEPPING(ros::NodeHandle&);
        void show_debug();
        void print();
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&);
};

#endif