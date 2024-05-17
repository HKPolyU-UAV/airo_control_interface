#include "airo_control/observer/rd3_observer.h"

RD3::RD3(ros::NodeHandle& nh){
    nh.getParam("airo_control_node/rd3_observer/k1",param.k1);
}

geometry_msgs::AccelStamped RD3::observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const double& ){
    
}