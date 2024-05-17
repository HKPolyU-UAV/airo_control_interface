#include "airo_control/controller/base_controller.h"

Eigen::Vector3d BASE_CONTROLLER::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Eigen::Vector3d euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.x(), euler.y(), euler.z());
    return euler;
}

geometry_msgs::Quaternion BASE_CONTROLLER::rpy2q(const Eigen::Vector3d& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.x(), euler.y(), euler.z());
    return quaternion;
}

float BASE_CONTROLLER::inverse_thrust_model(const double& a_z,const float& voltage,const Param& param,const ThrustModel& thrust_model){
    float thrust;

    if (param.enable_thrust_model) {
        thrust = ((sqrt((thrust_model.mass*a_z)/(thrust_model.K1*pow(voltage,thrust_model.K2))+pow(((1-thrust_model.K3)/(2*sqrt(thrust_model.K3))),2))-((1-thrust_model.K3)/(2*sqrt(thrust_model.K3))))/sqrt(thrust_model.K3));
    }
    else {
        thrust = (a_z/g)*param.hover_thrust;
    }

    if (thrust > 1.0) {
        ROS_ERROR("Thrust = %f",thrust);
        thrust = 1.0;
    }
    else if (thrust < 0.0) {
        thrust = 0.0;
    }

    return thrust;
}