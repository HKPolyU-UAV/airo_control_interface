#ifndef QUADROTOR_MPC_H
#define QUADROTOR_MPC_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>

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

    // Acados variables
    SolverInput acados_in;
    SolverOutput acados_out;
    int acados_status;   
    quadrotor_solver_capsule * mpc_capsule = quadrotor_acados_create_capsule();

    // Other variables
    tf::Quaternion tf_quaternion;
    int cout_counter = 0;

    mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist);

    public:

    QUADROTOR_MPC();
    mavros_msgs::AttitudeTarget run(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist, Eigen::VectorXd ref);
    mavros_msgs::AttitudeTarget run(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist, std::vector<Eigen::VectorXd> ref);
};

#endif