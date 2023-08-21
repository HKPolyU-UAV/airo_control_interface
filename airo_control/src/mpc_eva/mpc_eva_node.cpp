#include <airo_control/controller/mpc.h>

std::unique_ptr<BASE_CONTROLLER> controller;
geometry_msgs::PoseStamped local_pose;
geometry_msgs::TwistStamped local_twist;
geometry_msgs::AccelStamped local_accel;
airo_message::Reference controller_ref;
std::vector<double> solving_times;
mavros_msgs::AttitudeTarget attitude_target;
double total_time = 0;

double extract_yaw_from_quaternion(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    double phi,theta,psi;
    tf::Matrix3x3(tf_quaternion).getRPY(phi, theta, psi);
    return psi;
}

void command_cb(const airo_message::Reference::ConstPtr& msg){
    controller_ref = *msg;
    controller_ref.ref_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, extract_yaw_from_quaternion(msg->ref_pose.orientation));
    ros::Time start_time = ros::Time::now();
    attitude_target = controller->solve(local_pose,local_twist,local_accel,controller_ref);
    ros::Time end_time = ros::Time::now();
    solving_times.push_back((end_time - start_time).toSec());
    total_time += solving_times.back();

    double average_time = total_time / solving_times.size();
    double max_time = *std::max_element(solving_times.begin(), solving_times.end());
    double min_time = *std::min_element(solving_times.begin(), solving_times.end());

    // Output results
    std::cout << "-------------------------------"<<std::endl;
    std::cout << "Average solving time: " << average_time << " seconds" << std::endl;
    std::cout << "Max solving time: " << max_time << " seconds" << std::endl;
    std::cout << "Min solving time: " << min_time << " seconds" << std::endl;
    std::cout << "Total iterations: " << solving_times.size() <<std::endl;
    std::cout << "-------------------------------"<<std::endl;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header.stamp = msg->header.stamp;
    local_pose.pose = msg->pose;
}

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.header.stamp = msg->header.stamp;
    local_twist.twist = msg->twist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_eva_node");
    ros::NodeHandle nh;

    nh.setParam("airo_control_node/mpc/hover_thrust",0.5656428200006485);
    nh.setParam("airo_control_node/mpc/tau_phi",0.1742504945971047);
    nh.setParam("airo_control_node/mpc/tau_theta",0.1704274688906173);
    nh.setParam("airo_control_node/mpc/tau_psi",0.7410010081424614);
    nh.setParam("airo_control_node/mpc/show_debug",false);
    nh.setParam("airo_control_node/mpc/enable_preview",true);

    controller = std::make_unique<MPC>(nh);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",5,pose_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",5,twist_cb);
    ros::Subscriber command_sub = nh.subscribe<airo_message::Reference>("/airo_control/setpoint",1,command_cb);

    while(ros::ok()){
        ros::spin();
    }

    return 0;
}