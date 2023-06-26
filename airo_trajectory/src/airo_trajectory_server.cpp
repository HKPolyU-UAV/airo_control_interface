#include "airo_trajectory/airo_trajectory_server.h"

AIRO_TRAJECTORY_SERVER::AIRO_TRAJECTORY_SERVER(ros::NodeHandle& nh){
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&AIRO_TRAJECTORY_SERVER::pose_cb,this);
    fsm_info_sub = nh.subscribe<airo_control::FSMInfo>("/airo_control/fsm_info",10,&AIRO_TRAJECTORY_SERVER::fsm_info_cb,this);
    command_pub = nh.advertise<airo_control::Reference>("/airo_control/setpoint",10);
    takeoff_land_pub = nh.advertise<airo_control::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",10);
}

void AIRO_TRAJECTORY_SERVER::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void AIRO_TRAJECTORY_SERVER::fsm_info_cb(const airo_control::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        airo_control::Reference reference;
        reference.header.stamp = ros::Time::now();
        reference.ref_pose.resize(41);
        reference.ref_twist.resize(41);

        for (int i = 0; i < reference.ref_pose.size(); i++){
            reference.ref_pose[i].position.x = point.x;
            reference.ref_pose[i].position.y = point.y;
            reference.ref_pose[i].position.z = point.z;
            reference.ref_pose[i].orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(yaw_angle);
            reference.ref_twist[i].linear.x = 0.0;
            reference.ref_twist[i].linear.y = 0.0;
            reference.ref_twist[i].linear.z = 0.0;
        }
        
        command_pub.publish(reference);
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const geometry_msgs::Twist& twist, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        airo_control::Reference reference;
        reference.header.stamp = ros::Time::now();
        reference.ref_pose.resize(41);
        reference.ref_twist.resize(41);

        for (int i = 0; i < reference.ref_pose.size(); i++){
            reference.ref_pose[i].position.x = point.x;
            reference.ref_pose[i].position.y = point.y;
            reference.ref_pose[i].position.z = point.z;
            reference.ref_pose[i].orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(yaw_angle);
            reference.ref_twist[i].linear.x = twist.linear.x;
            reference.ref_twist[i].linear.y = twist.linear.y;
            reference.ref_twist[i].linear.z = twist.linear.z;
        }
        
        command_pub.publish(reference);
    }
}

bool AIRO_TRAJECTORY_SERVER::target_reached(const geometry_msgs::Point& msg){
    return sqrt(pow(msg.x - local_pose.pose.position.x,2)+pow(msg.y - local_pose.pose.position.y,2)
    +pow(msg.z - local_pose.pose.position.z,2)) < 0.5;
}

geometry_msgs::Quaternion AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(double yaw){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    return quaternion;
}

double AIRO_TRAJECTORY_SERVER::quaternion_to_yaw(const geometry_msgs::Quaternion& quaternion){
    double phi,theta,psi;
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(phi, theta, psi);
    return psi;
}

int AIRO_TRAJECTORY_SERVER::read_traj_file(const char* fileName, std::vector<std::vector<double>> &data){
    std::ifstream file(fileName);
    std::string line;
    int number_of_lines = 0;

    if (file.is_open())
    {
        while(getline(file, line)){
            number_of_lines++;
            std::istringstream linestream( line );
            std::vector<double> linedata;
            double number;

            while( linestream >> number ){
                linedata.push_back( number );
            }
            data.push_back( linedata );
        }

        file.close();
    }
    else
    {
        return 0;
    }

    return number_of_lines;
}

bool AIRO_TRAJECTORY_SERVER::takeoff(){
    if(fsm_info.is_landed == true){
        airo_control::TakeoffLandTrigger takeoff_trigger;
        ROS_INFO_THROTTLE(2.0, "[AIRo Trajectory] Sending takeoff trigger.");
        takeoff_trigger.takeoff_land_trigger = true; // Takeoff
        takeoff_trigger.header.stamp = ros::Time::now();
        takeoff_land_pub.publish(takeoff_trigger);
        return false;
    }
    else if(fsm_info.is_landed == false && fsm_info.is_waiting_for_command == true){
        ROS_INFO("[AIRo Trajectory] Vehicle already takeoff!");
        return true;
    }
}

bool AIRO_TRAJECTORY_SERVER::land(){
    if(fsm_info.is_landed == false && fsm_info.is_waiting_for_command == true){
        airo_control::TakeoffLandTrigger land_trigger;
        ROS_INFO_THROTTLE(2.0, "[AIRo Trajectory] Sending land trigger.");
        land_trigger.takeoff_land_trigger = false; // Land
        land_trigger.header.stamp = ros::Time::now();
        takeoff_land_pub.publish(land_trigger);
        return false;
    }
    else if(fsm_info.is_landed == true){
        ROS_INFO("[AIRo Trajectory] Vehicle has landed!");
        return true;
    }
}