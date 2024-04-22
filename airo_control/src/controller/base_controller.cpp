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


    if (param.enable_thrust_model) {

        // const float K1 = 1.450420984642443;
        // const float K2 = 1.143035702043988;
        // const float K3 = 0.422541015026179;
        const float K1 = 0.4513 ;
        const float K2 = 0.9442;
        const float K3 = 0.7631 ;
        const float K4 = 1.452 ;
        const double mass = 0.711;
        
        
              // thrust = ((sqrt(( mass * a_z) / (K1 * pow(voltage, K2)) + pow(((1 - K3) / (2 * sqrt(K3))), 2)) - ((1 - K3) / (2 * sqrt(K3)))) / sqrt(K3)) ;
                 thrust = (sqrt((mass * a_z )* K1) - K3) / (K2 * pow(voltage, K4));

        std::cout<<"enable thrust_model"<<std::endl;
        std::cout<<thrust<<std::endl;
       
    }
    else {
        thrust = (a_z/g)*param.hover_thrust;
        std::cout<<a_z<<std::endl;
        std::cout<<thrust<<std::endl;
    } 

    if (thrust > 1.0) {
        ROS_ERROR("Thrust = %f",thrust);
        thrust = 0.9999;
    }
    else if (thrust < 0.0) {
        ROS_ERROR("Thrust = %f",thrust);
        thrust = 0.0001;
    }

    return thrust;
}
