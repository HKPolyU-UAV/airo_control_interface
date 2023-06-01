#ifndef QUADROTOR_MPC_H
#define QUADROTOR_MPC_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>
#include <airo_px4/Reference.h>

#include <iostream>
#include <fstream>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "quadrotor_model/quadrotor_model.h"
#include "acados_solver_quadrotor.h"

class QUADROTOR_MPC{
    private:

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

    struct SolverInput{
        double x0[QUADROTOR_NX];
        double yref[QUADROTOR_N+1][QUADROTOR_NY];
    };

    struct SolverOutput{
        double u0[QUADROTOR_NU];
        double x1[QUADROTOR_NX];
        double status, kkt_res, cpu_time;
    };

    struct Euler{
        double phi;
        double theta;
        double psi;
    };

    // ROS message variables
    mavros_msgs::AttitudeTarget attitude_target;
    Euler local_euler;
    Euler target_euler;
    Euler ref_euler;

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    double acados_param[QUADROTOR_N+1][QUADROTOR_NP];  // hover_thrust, tau_phi, tau_theta, psi
    int acados_status;   
    quadrotor_solver_capsule *mpc_capsule = quadrotor_acados_create_capsule();

    // Other variables
    int cout_counter = 0;
    double delta_t = 0.025; // MPC sample time
    
    public:

    struct SolverParam{
        double hover_thrust;
        double tau_phi;
        double tau_theta;
        double tau_psi;
    };

    QUADROTOR_MPC();
    Euler q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Euler&);
    mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const airo_px4::Reference&, const SolverParam&);
};

#endif