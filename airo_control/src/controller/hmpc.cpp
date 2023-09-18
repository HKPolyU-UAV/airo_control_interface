#include "airo_control/controller/hmpc.h"

HMPC::HMPC(ros::NodeHandle& nh){
    // Get MPC parameters
    nh.getParam("airo_control_node/hmpc/hover_thrust",param.hover_thrust);
    nh.getParam("airo_control_node/hmpc/tau_phi",param.tau_phi);
    nh.getParam("airo_control_node/hmpc/tau_theta",param.tau_theta);
    nh.getParam("airo_control_node/hmpc/show_debug",param.show_debug);
    nh.getParam("airo_control_node/hmpc/enable_preview",param.enable_preview);

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


void HMPC::show_debug(){
    if (param.show_debug){
        if(debug_counter > 2){ //reduce cout rate
            HMPC::print();
            debug_counter = 0;
        }
        else{
            debug_counter++;
        }
    }
}

void HMPC::print(){
    std::cout << "------------------------------------------------------------------------------" << std::endl;
    std::cout << "x_ref:      " << acados_in.yref[0][0] << "\ty_ref:      " << acados_in.yref[0][1] << "\tz_ref:         " << acados_in.yref[0][2] << std::endl;
    std::cout << "x_gt:       " << acados_in.x0[x] << "\ty_gt:       " << acados_in.x0[y] << "\tz_gt:          " << acados_in.x0[z] << std::endl;
    std::cout << "theta_cmd:  " << target_euler.y() << "\tphi_cmd:    " << target_euler.x() <<  "\tpsi_cmd:       " << target_euler.z() << std::endl;
    std::cout << "theta_gt:   " << current_euler.y() << "\tphi_gt:     " << current_euler.x() <<  "\tpsi_gt:        " << current_euler.z() << std::endl;
    std::cout << "thrust_cmd: " << attitude_target.thrust << "\tsolve_time: "<< acados_out.cpu_time  << "\tacados_status: " << acados_out.status << std::endl;
    std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
    std::cout << "------------------------------------------------------------------------------" << std::endl;
}

mavros_msgs::AttitudeTarget HMPC::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::Reference& ref){
    // Resize ref to fit prediction horizon
    airo_message::ReferencePreview ref_preview;
    ref_preview.header = ref.header;
    ref_preview.ref_pose.resize(QUADROTOR_N+1);
    ref_preview.ref_twist.resize(QUADROTOR_N+1);
    ref_preview.ref_accel.resize(QUADROTOR_N+1);
    for (int i = 0; i < QUADROTOR_N+1; i++){
        ref_preview.ref_pose[i] = ref.ref_pose;
        ref_preview.ref_twist[i] = ref.ref_twist;
        ref_preview.ref_accel[i] = ref.ref_accel;
    }
    return HMPC::solve(current_pose,current_twist,current_accel,ref_preview);
}

mavros_msgs::AttitudeTarget HMPC::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::ReferencePreview& ref_preview){
    // Set reference
    ref_euler = BASE_CONTROLLER::q2rpy(ref_preview.ref_pose[0].orientation);
    for (int i = 0; i < QUADROTOR_N+1; i++){
        acados_in.yref[i][0] = ref_preview.ref_pose[i].position.x;
        acados_in.yref[i][1] = ref_preview.ref_pose[i].position.y;
        acados_in.yref[i][2] = ref_preview.ref_pose[i].position.z;
        acados_in.yref[i][3] = ref_preview.ref_twist[i].linear.x;
        acados_in.yref[i][4] = ref_preview.ref_twist[i].linear.y;
        acados_in.yref[i][5] = ref_preview.ref_twist[i].linear.z;
        acados_in.yref[i][6] = 0;
        acados_in.yref[i][7] = 0;
        acados_in.yref[i][8] = param.hover_thrust;
        acados_in.yref[i][9] = 0;
        acados_in.yref[i][10] = 0;
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }

    // Set initial states
    current_euler = BASE_CONTROLLER::q2rpy(current_pose.pose.orientation);
    acados_in.x0[x] = current_pose.pose.position.x;
    acados_in.x0[y] = current_pose.pose.position.y;
    acados_in.x0[z] = current_pose.pose.position.z;
    acados_in.x0[u] = current_twist.twist.linear.x;
    acados_in.x0[v] = current_twist.twist.linear.y;
    acados_in.x0[w] = current_twist.twist.linear.z;
    acados_in.x0[phi] = current_euler.x();
    acados_in.x0[theta] = current_euler.y();
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, 0, "ubx", acados_in.x0);

    // Set parameters
    for (int i = 0; i < QUADROTOR_N+1; i++){
        acados_param[i][0] = param.hover_thrust;
        acados_param[i][1] = param.tau_phi;
        acados_param[i][2] = param.tau_theta;
        acados_param[i][3] = 0;
        // acados_param[i][3] = current_euler.z();
        quadrotor_acados_update_params(mpc_capsule,i,acados_param[i],QUADROTOR_NP);
    }

    // Solve OCP
    acados_status = quadrotor_acados_solve(mpc_capsule);
    if (acados_status != 0){
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
        HMPC::print();
    }
    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    attitude_target.thrust = acados_out.u0[0]; 
    target_euler.x() = acados_out.u0[1];
    target_euler.y() = acados_out.u0[2];
    target_euler.z() = 0.0;

    geometry_msgs::Quaternion target_quaternion = BASE_CONTROLLER::rpy2q(target_euler);
    attitude_target.orientation = target_quaternion;

    HMPC::show_debug();
    return attitude_target;
}

double HMPC::get_hover_thrust(){
    return param.hover_thrust;
}