#ifndef MPC_H
#define MPC_H

#include <iostream>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "quadrotor_model/quadrotor_model.h"
#include "acados_solver_quadrotor.h"

#include "airo_control/controller/base_controller.h"
#include "airo_message/ReferencePreview.h"

class MPC : public BASE_CONTROLLER{
    private:
        struct Param : public BASE_CONTROLLER::Param{
            double tau_phi;
            double tau_theta;
            double tau_psi;
            bool enable_preview;
            std::vector<double> diag_cost_x,diag_cost_u,diag_cost_xn;
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

        struct SolverInput{
            double x0[QUADROTOR_NX];
            double yref[QUADROTOR_N+1][QUADROTOR_NY];
        };

        struct SolverOutput{
            double u0[QUADROTOR_NU];
            double x1[QUADROTOR_NX];
            double status,kkt_res, cpu_time;
        };

        ros::Publisher debug_pub;

        // Acados variables
        SolverInput acados_in;
        SolverOutput acados_out;
        double acados_param[QUADROTOR_N+1][QUADROTOR_NP];  // hover_thrust, tau_phi, tau_theta, psi
        int acados_status;   
        quadrotor_solver_capsule *mpc_capsule = quadrotor_acados_create_capsule();
        
    public:
        Param param;
        MPC(ros::NodeHandle&);
        void pub_debug();
        void print_debug();
        double get_hover_thrust();
        bool set_intermediate_weights(const std::vector<double>&,const std::vector<double>&);
        bool set_terminal_weights(const std::vector<double>&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&, const geometry_msgs::Vector3Stamped&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::ReferencePreview&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::ReferencePreview&, const geometry_msgs::Vector3Stamped&);
};

#endif