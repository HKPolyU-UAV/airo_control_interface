#include "airo_control/observer/rd3_observer.h"

RD3::RD3(ros::NodeHandle& nh)
{
    std::vector<double> th(3);
    std::vector<double> omega(3);
    nh.getParam("airo_control_node/rd3_observer/alpha1", param.alpha(0));
    nh.getParam("airo_control_node/rd3_observer/alpha2", param.alpha(1));
    nh.getParam("airo_control_node/rd3_observer/alpha3", param.alpha(2));
    nh.getParam("airo_control_node/rd3_observer/beta1", param.beta(0));
    nh.getParam("airo_control_node/rd3_observer/beta2", param.beta(1));
    nh.getParam("airo_control_node/rd3_observer/beta3", param.beta(2));
    nh.getParam("airo_control_node/rd3_observer/dt", param.dt);
    nh.getParam("airo_control_node/rd3_observer/g", param.g);
    nh.getParam("airo_control_node/rd3_observer/g", param.g);
    nh.getParam("airo_control_node/rd3_observer/dead_zone", param.dead_zone);
    nh.getParam("airo_control_node/rd3_observer/omega", omega);
    param.threshold = Eigen::Vector3d(th[0], th[1], th[2]);

    double m1n1 = omega[0] + omega[1] + omega[2];
    double m2n2 = omega[0] * omega[1] + omega[0] * omega[2] + omega[1] * omega[2];
    double m3n3 = omega[0] * omega[1] * omega[2];
    param.m1 = m1n1 * ones();
    param.m2 = m2n2 * ones();
    param.m3 = m3n3 * ones();
    param.n1 = m1n1 * ones();
    param.n2 = m2n2 * ones();
    param.n3 = m3n3 * ones();
}

double RD3::sgn(double x)
{
    if (x < 0)          return -1.;
    else if (x == 0)    return 0.;
    else                return 1.;
}

Eigen::Vector3d RD3::ones(void)
{
    return Eigen::Vector3d(1., 1., 1.);
}

Eigen::Vector3d RD3::sig(Eigen::Vector3d x, double a)
{
    Eigen::Vector3d res(0., 0., 0.);
    for (int i = 0; i < 3; i++)
        res(i) = pow(fabs(x(i)), a) * sgn(x(i));
    return res;
}

Eigen::Vector3d RD3::fal(Eigen::Vector3d xi, double a)
{
    Eigen::Vector3d res(0., 0., 0.);
    for (int i = 0; i < 3; i++)
    {
        if (fabs(xi(i)) <= param.threshold(i))
            res(i) = xi(i) / (pow(param.threshold(i), fabs(1 - a)));
        else
            res(i) = pow(xi(i), a) * sgn(xi(i));
    }
    return res;
}

void RD3::SetInit(Eigen::Vector3d e0, Eigen::Vector3d de0, Eigen::Vector3d sys_dy, bool use)
{
    if (use)
        z1 = e0;
        z2 = de0;
        z3 = 0 * ones();
        dz1 = z2;
        dz2 = z3 + sys_dy;
        dz3 = 0 * ones();
}

geometry_msgs::AccelStamped RD3::observe(const geometry_msgs::PoseStamped& ros_pos, const geometry_msgs::TwistStamped& ros_vel, const geometry_msgs::AccelStamped& ros_acc, const double& tpm)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(ros_pos.pose.orientation, quat);
    double phi, theta, psi;
    tf::Matrix3x3(quat).getEulerYPR(phi, theta, psi);

    geometry_msgs::Point _pos = ros_pos.pose.position;
    Eigen::Vector3d pos(_pos.x, _pos.y, _pos.z);
    Eigen::Vector3d A = tpm * Eigen::Vector3d(cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi),
                                              cos(phi) * sin(psi) * sin(theta) - sin(phi) * cos(psi),
                                              cos(phi) * cos(theta))
                                              - Eigen::Vector3d(0., 0., param.g);
    Eigen::Vector3d obs_e = Eigen::Vector3d() - z1;
    dz1 = param.m1.cwiseProduct(sig(obs_e, param.alpha(0))) + param.n1.cwiseProduct(sig(obs_e, param.beta(0))) + z2;
    dz2 = param.m2.cwiseProduct(sig(obs_e, param.alpha(1))) + param.n2.cwiseProduct(sig(obs_e, param.beta(1))) + z3 + A;
    dz3 = param.m3.cwiseProduct(sig(obs_e, param.alpha(2))) + param.n3.cwiseProduct(sig(obs_e, param.beta(2)));
    z1 += dz1 * param.dt;
    z2 += dz2 * param.dt;
    z3 += dz3 * param.dt;

    geometry_msgs::AccelStamped res;
    res.accel.linear.x = z3(0);
    res.accel.linear.y = z3(1);
    res.accel.linear.z = z3(2);
    return res;
}