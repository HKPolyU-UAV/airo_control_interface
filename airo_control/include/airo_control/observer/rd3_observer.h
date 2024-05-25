#ifndef RD3_H
#define RD3_H

#include "airo_control/observer/base_observer.h"
#include <eigen3/Eigen/Dense>

class RD3 : public BASE_OBSERVER
{
    protected:
        struct Param{
            Eigen::Vector3d alpha, beta, threshold;
            double dt, g, dead_zone;
            Eigen::Vector3d m1, m2, m3;
            Eigen::Vector3d n1, n2, n3;
        };

        Param param;
    private:
        double sgn(double x);
        Eigen::Vector3d ones(void);
        Eigen::Vector3d sig(Eigen::Vector3d x, double a);
        Eigen::Vector3d fal(Eigen::Vector3d xi, double a);
        Eigen::Vector3d z1;
        Eigen::Vector3d z2;
        Eigen::Vector3d z3;
        Eigen::Vector3d dz1;
        Eigen::Vector3d dz2;
        Eigen::Vector3d dz3;
    public:
        RD3(ros::NodeHandle&);
        void SetInit(Eigen::Vector3d e0, Eigen::Vector3d de0, Eigen::Vector3d sys_dy, bool use);
        geometry_msgs::AccelStamped observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&, const geometry_msgs::AccelStamped&, const double&);
};

#endif