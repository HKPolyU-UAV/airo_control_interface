#ifndef OBSERVER_EKF_H
#define OBSERVER_EKF_H

#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/ApplyBodyWrench.h>

#include "acados_solver_quadrotor.h"

#include "airo_control/controller/base_controller.h"
#include "airo_message/ReferencePreview.h"

using namespace Eigen;

class OBSERVER_EKF: public BASE_CONTROLLER{
    private:
        struct Param: public BASE_CONTROLLER::Param{
            double tau_phi;
            double tau_theta;
            double tau_psi;
        };

        double g = 9.81;
        double dt = 0.05;

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
            p = 9,
            q = 10,
        };

        enum ControlInputs{
            thrust = 0,
            phi_cmd = 1,
            theta_cmd = 2,
            psi_cmd = 3,
        };

        struct SolverInput{
            double x0[QUADROTOR_NX];
            double yref[QUADROTOR_N+1][QUADROTOR_NY];
        };


    public:
        Param param;
        OBSERVER_EKF(ros::NodeHandle&);

};

#endif