#include "airo_control/controller/mpc.h"

MPC::MPC(ros::NodeHandle& nh){
    // Get MPC parameters
    nh.getParam("airo_control_node/mpc/hover_thrust",param.hover_thrust);
    nh.getParam("airo_control_node/mpc/tau_phi",param.tau_phi);
    nh.getParam("airo_control_node/mpc/tau_theta",param.tau_theta);
    nh.getParam("airo_control_node/mpc/tau_psi",param.tau_psi);
    nh.getParam("airo_control_node/mpc/pub_debug",param.pub_debug);
    nh.getParam("airo_control_node/mpc/enable_preview",param.enable_preview);
    nh.getParam("airo_control_node/mpc/diag_cost_x",param.diag_cost_x);
    nh.getParam("airo_control_node/mpc/diag_cost_u",param.diag_cost_u);
    nh.getParam("airo_control_node/mpc/diag_cost_xn",param.diag_cost_xn);

    // Set publishers
    debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/airo_control/mpc/debug",1);

    // Initialize MPC
    int create_status = 1;
    create_status = quadrotor_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    for(unsigned int i=0; i < QUADROTOR_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < QUADROTOR_NX; i++) acados_in.x0[i] = 0.0;

    // Set weights
    set_intermediate_weights(param.diag_cost_x,param.diag_cost_u);
    set_terminal_weights(param.diag_cost_xn);
}

mavros_msgs::AttitudeTarget MPC::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::Reference& ref){
    geometry_msgs::Vector3Stamped force_disturbance;
    force_disturbance.vector.x = 0;
    force_disturbance.vector.y = 0;
    force_disturbance.vector.z = 0;
    return MPC::solve(current_pose,current_twist,current_accel,ref,force_disturbance);
}

mavros_msgs::AttitudeTarget MPC::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::Reference& ref, const geometry_msgs::Vector3Stamped& force_disturbance){
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
    return MPC::solve(current_pose,current_twist,current_accel,ref_preview,force_disturbance);
}

mavros_msgs::AttitudeTarget MPC::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::ReferencePreview& ref_preview){
    geometry_msgs::Vector3Stamped force_disturbance;
    force_disturbance.vector.x = 0;
    force_disturbance.vector.y = 0;
    force_disturbance.vector.z = 0;
    return MPC::solve(current_pose,current_twist,current_accel,ref_preview,force_disturbance);
}

mavros_msgs::AttitudeTarget MPC::solve(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::TwistStamped& current_twist, const geometry_msgs::AccelStamped& current_accel, const airo_message::ReferencePreview& ref_preview, const geometry_msgs::Vector3Stamped& force_disturbance){
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

        acados_param[i][4] = force_disturbance.vector.x;
        acados_param[i][5] = force_disturbance.vector.y;
        acados_param[i][6] = force_disturbance.vector.z;

        // For yaw prediction
        if (i == 0){
            acados_param[i][3] = current_euler.z();
        }
        else{
            Eigen::Vector3d dummy_euler = BASE_CONTROLLER::q2rpy(ref_preview.ref_pose[i-1].orientation);
            if (dummy_euler.z() - acados_param[i-1][3] > M_PI){
                dummy_euler.z() = dummy_euler.z() - 2*M_PI;
            }
            else if (dummy_euler.z() - acados_param[i-1][3] < -M_PI){
                dummy_euler.z() = dummy_euler.z() + 2*M_PI;
            }
            acados_param[i][3] = acados_param[i-1][3] + 1.0/QUADROTOR_N * (dummy_euler.z() - acados_param[i-1][3]) / param.tau_psi;
        }
        quadrotor_acados_update_params(mpc_capsule,i,acados_param[i],QUADROTOR_NP);
    }
    
    // Solve OCP
    acados_status = quadrotor_acados_solve(mpc_capsule);
    if (acados_status != 0){
        ROS_INFO_STREAM("acados returned status " << acados_status << std::endl);
        MPC::print_debug();
    }
    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_config, mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);
    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);

    attitude_target.thrust = acados_out.u0[0]; 
    target_euler.x() = acados_out.u0[1];
    target_euler.y() = acados_out.u0[2];
    target_euler.z() = ref_euler.z();

    geometry_msgs::Quaternion target_quaternion = BASE_CONTROLLER::rpy2q(target_euler);
    attitude_target.orientation = target_quaternion;

    if (param.pub_debug){
        pub_debug();
    }

    return attitude_target;
}

double MPC::get_hover_thrust(){
    return param.hover_thrust;
}

bool MPC::set_intermediate_weights(const std::vector<double>&diag_weight_x, const std::vector<double>&diag_weight_u){
    if (diag_weight_x.size() != QUADROTOR_NX || diag_weight_u.size() != QUADROTOR_NU){
        return false;
    }

	double w[(QUADROTOR_NX+QUADROTOR_NU)*(QUADROTOR_NX+QUADROTOR_NU)];
	for (int j = 0; j < (QUADROTOR_NX+QUADROTOR_NU)*(QUADROTOR_NX+QUADROTOR_NU); j++){
		w[j] = 0.0;
    }
	for (int j = 0; j < QUADROTOR_NX; j++){
        w[j + (QUADROTOR_NX+QUADROTOR_NU) * j] = diag_weight_x[j];
    }
	for (int j = 0; j < QUADROTOR_NU; j++){
        w[QUADROTOR_NX+j + (QUADROTOR_NX+QUADROTOR_NU) * (QUADROTOR_NX+j)] = diag_weight_u[j];
    }
		
    for (int i = 0; i < QUADROTOR_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in,i,"W",w);
    }
    return true;
}

bool MPC::set_terminal_weights(const std::vector<double>&diag_weight_xn){
    if (diag_weight_xn.size() != QUADROTOR_NX){
        return false;
    }

    double w_n[QUADROTOR_NX*QUADROTOR_NX];
	for (int j = 0; j < QUADROTOR_NX*QUADROTOR_NX; j++){
        w_n[j] = 0.0;
    }
	for (int j = 0; j < QUADROTOR_NX; j++){
        w_n[j + QUADROTOR_NX * j] = diag_weight_xn[j];
    }

    ocp_nlp_cost_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in,QUADROTOR_N,"W",w_n);
    return true;
}

void MPC::pub_debug(){
    std_msgs::Float64MultiArray debug_msg;
    debug_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    debug_msg.layout.dim[0].size = QUADROTOR_N + 4;
    debug_msg.layout.dim[0].stride = 1;
    debug_msg.data.clear();
    debug_msg.data.push_back(acados_out.status);
    debug_msg.data.push_back(acados_out.kkt_res);
    debug_msg.data.push_back(acados_out.cpu_time);
    for (size_t i = 0; i < QUADROTOR_N + 1; i++){
        debug_msg.data.push_back(acados_param[i][3]);
    }

    debug_pub.publish(debug_msg);
}

void MPC::print_debug(){
    std::cout << "------------------------------------------------------------------------------" << std::endl;
    std::cout << "x_ref:      " << acados_in.yref[0][0] << "\ty_ref:      " << acados_in.yref[0][1] << "\tz_ref:         " << acados_in.yref[0][2] << std::endl;
    std::cout << "x_gt:       " << acados_in.x0[x] << "\ty_gt:       " << acados_in.x0[y] << "\tz_gt:          " << acados_in.x0[z] << std::endl;
    std::cout << "theta_cmd:  " << target_euler.y() << "\tphi_cmd:    " << target_euler.x() <<  "\tpsi_cmd:       " << target_euler.z() << std::endl;
    std::cout << "theta_gt:   " << current_euler.y() << "\tphi_gt:     " << current_euler.x() <<  "\tpsi_gt:        " << current_euler.z() << std::endl;
    std::cout << "thrust_cmd: " << attitude_target.thrust << "\tsolve_time: "<< acados_out.cpu_time  << "\tacados_status: " << acados_out.status << std::endl;
    std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
    std::cout << "------------------------------------------------------------------------------" << std::endl;
}