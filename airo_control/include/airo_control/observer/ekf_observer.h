#ifndef EKF_H
#define EKF_H

#include "airo_control/observer/base_observer.h"

class EKF : public BASE_OBSERVER{
    protected:

    public:
        EKF(ros::NodeHandle&);
        geometry_msgs::AccelStamped observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const double&);
};

#endif