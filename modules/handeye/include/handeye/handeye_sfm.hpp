#ifndef HANDEYE_SFM_HPP
#define HANDEYE_SFM_HPP
#include <vector>
#include <manif/manif.h>
#include <Eigen/Core>

namespace calibration_toolkit
{

    static const manif::SE3Tangentd a_se3 = manif::SE3Tangentd((Eigen::Matrix<double, 6, 1>() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished());
    Eigen::Matrix<double, 6, 6> J66_Bm_Xm(const manif::SE3d &Bm);

    Eigen::Matrix<double, 3, 6> J36_Ipx(const Eigen::Vector3d &p);

    Eigen::Vector2d proj(const Eigen::Vector3d &p);

    Eigen::Matrix<double, 6, 12> J612_B_X(const manif::SE3d &B1, const manif::SE3d &B2);

    Eigen::Matrix<double, 2, 3> J23_pi(const Eigen::Vector3d &p);

    void J36(const manif::SE3d &B, const manif::SE3d &L, const Eigen::Vector3d &p_obj,
             Eigen::Matrix<double, 3, 6> &J_B, Eigen::Matrix<double, 3, 6> &J_L);

    manif::SE3d Bm(const manif::SE3d &Xm, double theta);

    Eigen::Matrix4d B12(const manif::SE3d &X1, const manif::SE3d &X2, double theta1, double theta2);

    void J218_p_XL(const manif::SE3d &B1, const manif::SE3d &B2, const manif::SE3d &L,
                   const Eigen::Vector3d &p_obj, Eigen::Vector2d &p_obj_proj,
                   Eigen::Matrix<double, 2, 12> &Jij_X, Eigen::Matrix<double, 2, 6> &Jij_L);

    void reprojection(const Eigen::Matrix2Xd &p_img, const Eigen::Matrix3Xd &p_obj,
                      const manif::SE3d &X1, const manif::SE3d &X2,
                      double theta1, double theta2, const manif::SE3d &L, Eigen::VectorXd &ei, Eigen::MatrixXd &Ji);

}

#endif