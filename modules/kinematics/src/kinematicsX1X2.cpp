#include "kinematics/kinematics.hpp"

namespace calibration_toolkit
{
    const manif::SE3Tangentd KinematicsX1X2::a_se3 = manif::SE3Tangentd((Eigen::Matrix<double, 6, 1>() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished());
    const manif::SO3Tangentd KinematicsX1X2::a_so3 = manif::SO3Tangentd(Eigen::Vector3d(0, 0, 1));

    KinematicsX1X2::KinematicsX1X2(Eigen::Matrix4d *X)
    {
        maxDoF = 2;
        for (size_t m = 0; m < maxDoF; m++)
        {
            X_SE3[m] = manif::SE3d(X[m].block<3, 1>(0, 3), Eigen::AngleAxisd(X[m].block<3, 3>(0, 0)));
            X_SO3[m] = manif::SO3d(Eigen::AngleAxisd(X[m].block<3, 3>(0, 0)));
        }
    };

    void KinematicsX1X2::reset(Eigen::Matrix4d *X)
    {
        maxDoF = 2;
        for (size_t m = 0; m < maxDoF; m++)
        {
            X_SE3[m] = manif::SE3d(X[m].block<3, 1>(0, 3), Eigen::AngleAxisd(X[m].block<3, 3>(0, 0)));
            X_SO3[m] = manif::SO3d(Eigen::AngleAxisd(X[m].block<3, 3>(0, 0)));
        }
    };

    void KinematicsX1X2::reset()
    {
        maxDoF = 2;
        X_SE3[0] = manif::SE3d(Eigen::Vector3d::Zero(), Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()));
        X_SE3[1] = manif::SE3d(Eigen::Vector3d::Zero(), Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));
        X_SO3[0] = manif::SO3d(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()));
        X_SO3[1] = manif::SO3d(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));
    };

    manif::SE3d KinematicsX1X2::FK_SE3(const Eigen::VectorXd &t) const
    {
        return (X_SE3[0].inverse() * (a_se3 * t(0)).exp() * X_SE3[0] * X_SE3[1].inverse() * (a_se3 * t(1)).exp() * X_SE3[1]);
    };

    Eigen::Matrix4d KinematicsX1X2::FK(const Eigen::VectorXd &t) const
    {
        return FK_SE3(t).transform();
    };

    Eigen::Matrix4d KinematicsX1X2::FK(const Eigen::VectorXd &t_from, const Eigen::VectorXd &t_to) const
    {
        return FK_SE3(t_from, t_to).transform();
    };
    manif::SE3d KinematicsX1X2::FK_SE3(const Eigen::VectorXd &t_from, const Eigen::VectorXd &t_to) const
    {
        return FK_SE3(t_to).inverse() * FK_SE3(t_from);
    };

}