#ifndef HMPC_REDUCED_H
#define HMPC_REDUCED_H

#include <iostream>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "quadrotor_tailsitter_hybrid_reduced_model/quadrotor_tailsitter_hybrid_reduced_model.h"
#include "acados_solver_quadrotor_tailsitter_hybrid_reduced.h"

#include "airo_control/controller/base_controller.h"
#include "airo_message/ReferencePreview.h"

class HMPC_REDUCED : public BASE_CONTROLLER{
    private:
        struct Param : public BASE_CONTROLLER::Param{
            double tau_phi;
            double tau_theta;
            bool enable_preview;
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
            double x0[QUADROTOR_TAILSITTER_HYBRID_REDUCED_NX];
            double yref[QUADROTOR_TAILSITTER_HYBRID_REDUCED_N+1][QUADROTOR_TAILSITTER_HYBRID_REDUCED_NY];
        };

        struct SolverOutput{
            double u0[QUADROTOR_TAILSITTER_HYBRID_REDUCED_NU];
            double x1[QUADROTOR_TAILSITTER_HYBRID_REDUCED_NX];
            double status, kkt_res, cpu_time;
        };

        // Acados variables
        SolverInput acados_in;
        SolverOutput acados_out;
        double acados_param[QUADROTOR_TAILSITTER_HYBRID_REDUCED_N+1];  // hover_thrust, tau_phi, tau_theta, psi
        int acados_status;   
        quadrotor_tailsitter_hybrid_reduced_solver_capsule *mpc_capsule = quadrotor_tailsitter_hybrid_reduced_acados_create_capsule();
        int debug_counter = 0;
        
    public:
        Param param;
        HMPC_REDUCED(ros::NodeHandle&);
        void show_debug();
        void print();
        double get_hover_thrust();
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::Reference&);
        mavros_msgs::AttitudeTarget solve(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const airo_message::ReferencePreview&);
};

#endif