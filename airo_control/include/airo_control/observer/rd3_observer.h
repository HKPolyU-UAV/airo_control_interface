#ifndef RD3_H
#define RD3_H

#include "airo_control/observer/base_observer.h"

class RD3 : public BASE_OBSERVER{
    protected:

    public:
        RD3(ros::NodeHandle&);
        geometry_msgs::AccelStamped observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const double&);
};

#endif