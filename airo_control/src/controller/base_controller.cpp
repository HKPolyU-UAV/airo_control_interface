#include "airo_control/controller/base_controller.h"
#include <cmath> 
#include <iostream>

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

float BASE_CONTROLLER::inverse_thrust_model(const double& a_z,const float& voltage,const Param& param,const ThrustModel& thrust_model)
{
    float thrust;
    float voltage_2 = 16.8; // 4s battery fully charged

    if (param.enable_thrust_model) {

            
            // thrust = (sqrt(((1 * a_z)/4) / (K1 * pow(voltage_2, K2)) + pow(((1 - K3) / (2 * sqrt(K3))), 2)) - ((1 - K3) / (2 * sqrt(K3)))) / sqrt(K3);
            double thrust = (sqrt(((thrust_model.mass * a_z)/ 4) / (thrust_model.K1 * pow(voltage, thrust_model.K2)) + pow((1 - thrust_model.K3) / (2 * sqrt(thrust_model.K3)), 2)) - ((1 - thrust_model.K3) / (2 * sqrt(thrust_model.K3)))) / sqrt(thrust_model.K3);
 
        // std::cout<<a_z<<std::endl;
        std::cout<<"enable thrust_model"<<std::endl;
        std::cout<<thrust<<std::endl;

    }
    else {
        thrust = (a_z/g)*param.hover_thrust;
        std::cout<<a_z<<std::endl;
        std::cout<<thrust<<std::endl;
    } 

    // if (thrust > 1.0) {
    //     ROS_ERROR("Thrust = %f",thrust);
    //     thrust = 0.9999;
    // }
    // else if (thrust < 0.0) {
    //     ROS_ERROR("Thrust = %f",thrust);
    //     thrust = 0.0001;
    // }

    return thrust;
}