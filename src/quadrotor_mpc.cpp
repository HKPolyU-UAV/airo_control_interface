#include "airo_px4/quadrotor_mpc.h"

QUADROTOR_MPC::QUADROTOR_MPC(){
    // Initialize MPC
    int create_status = 1;
    create_status = quadrotor_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    for(unsigned int i=0; i < QUADROTOR_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < QUADROTOR_NX; i++) acados_in.x0[i] = 0.0;
}

QUADROTOR_MPC::Euler QUADROTOR_MPC::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}
geometry_msgs::Quaternion QUADROTOR_MPC::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

mavros_msgs::AttitudeTarget QUADROTOR_MPC::solve(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& twist, const airo_px4::Reference& ref, const SolverParam& param){
    // Set reference
    ref_euler = q2rpy(ref.ref_pose[0].orientation);
    for (int i = 0; i < QUADROTOR_N+1; i++){
        acados_in.yref[i][0] = ref.ref_pose[i].position.x;
        acados_in.yref[i][1] = ref.ref_pose[i].position.y;
        acados_in.yref[i][2] = ref.ref_pose[i].position.z;
        acados_in.yref[i][3] = ref.ref_twist[i].linear.x;
        acados_in.yref[i][4] = ref.ref_twist[i].linear.y;
        acados_in.yref[i][5] = ref.ref_twist[i].linear.z;
        acados_in.yref[i][6] = 0;
        acados_in.yref[i][7] = 0;
        acados_in.yref[i][8] = param.hover_thrust;
        acados_in.yref[i][9] = 0;
        acados_in.yref[i][10] = 0;
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }

    // Set initial states
    local_euler = q2rpy(pose.pose.orientation);
    acados_in.x0[x] = pose.pose.position.x;
    acados_in.x0[y] = pose.pose.position.y;
    acados_in.x0[z] = pose.pose.position.z;
    acados_in.x0[u] = twist.twist.linear.x;
    acados_in.x0[v] = twist.twist.linear.y;
    acados_in.x0[w] = twist.twist.linear.z;
    acados_in.x0[phi] = local_euler.phi;
    acados_in.x0[theta] = local_euler.theta;
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);

    // Set parameters
    for (int i = 0; i < QUADROTOR_N+1; i++){
        acados_param[i][0] = param.hover_thrust;
        acados_param[i][1] = param.tau_phi;
        acados_param[i][2] = param.tau_theta;
        if (i == 0){
            acados_param[i][3] = local_euler.psi;
        }
        else{
            Euler dummy_euler = q2rpy(ref.ref_pose[i-1].orientation); // For yaw prediction
            if (dummy_euler.psi - acados_param[i-1][3] > M_PI){
                dummy_euler.psi = dummy_euler.psi - 2*M_PI;
            }
            else if (dummy_euler.psi - acados_param[i-1][3] < -M_PI){
                dummy_euler.psi = dummy_euler.psi + 2*M_PI;
            }
            acados_param[i][3] = acados_param[i-1][3] + delta_t * (dummy_euler.psi - acados_param[i-1][3]) / param.tau_psi;
        }
        quadrotor_acados_update_params(mpc_capsule,i,acados_param[i],QUADROTOR_NP);
    }

    // Solve OCP
    acados_status = quadrotor_acados_solve(mpc_capsule);
    if (acados_status != 0){
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
    }
    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    attitude_target.thrust = acados_out.u0[0];  
    target_euler.phi = acados_out.u0[1];
    target_euler.theta = acados_out.u0[2];
    target_euler.psi = ref_euler.psi;

    geometry_msgs::Quaternion target_quaternion = rpy2q(target_euler);
    attitude_target.orientation.w = target_quaternion.w;
    attitude_target.orientation.x = target_quaternion.x;
    attitude_target.orientation.y = target_quaternion.y;
    attitude_target.orientation.z = target_quaternion.z;

    // /*Mission information cout**********************************************/        
    // if(cout_counter > 2){ //reduce cout rate
    //     std::cout << "------------------------------------------------------------------------------" << std::endl;
    //     std::cout << "x_ref:      " << acados_in.yref[0][0] << "\ty_ref:      " << acados_in.yref[0][1] << "\tz_ref:         " << acados_in.yref[0][2] << std::endl;
    //     std::cout << "x_gt:       " << acados_in.x0[x] << "\ty_gt:       " << acados_in.x0[y] << "\tz_gt:          " << acados_in.x0[z] << std::endl;
    //     std::cout << "theta_cmd:  " << target_euler.theta << "\tphi_cmd:    " << target_euler.phi <<  "\tpsi_cmd:       " << target_euler.psi << std::endl;
    //     std::cout << "theta_gt:   " << local_euler.theta << "\tphi_gt:     " << local_euler.phi <<  "\tpsi_gt:        " << local_euler.psi << std::endl;
    //     std::cout << "thrust_cmd: " << attitude_target.thrust << "\tsolve_time: "<< acados_out.cpu_time  << "\tacados_status: " << acados_out.status << std::endl;
    //     std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
    //     std::cout << "------------------------------------------------------------------------------" << std::endl;
    //     cout_counter = 0;
    // }
    // else{
    //     cout_counter++;
    // }

    return attitude_target;
}