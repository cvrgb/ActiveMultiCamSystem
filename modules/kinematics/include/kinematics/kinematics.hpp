#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP
#include <vector>
#include <Eigen/Core>
#include <manif/manif.h>

namespace calibration_toolkit
{
    class KinematicsBase
    {
    public:
        virtual Eigen::Matrix4d FK(const Eigen::VectorXd &encoder) const = 0;
        virtual Eigen::VectorXd IK(const Eigen::Matrix4d &pose_b_e) const = 0;
        size_t maxDoF = 1;
    };
    class KinematicsX1X2 : public KinematicsBase
    {
    public:
        static const manif::SE3Tangentd a_se3;
        static const manif::SO3Tangentd a_so3;

        KinematicsX1X2(Eigen::Matrix4d *X);
        void reset();
        void reset(Eigen::Matrix4d *X);
        Eigen::Matrix4d FK(const Eigen::VectorXd &thetas) const;
        Eigen::Matrix4d FK(const Eigen::VectorXd &thetas_from, const Eigen::VectorXd &thetas_to) const;
        manif::SE3d FK_SE3(const Eigen::VectorXd &thetas) const;
        manif::SE3d FK_SE3(const Eigen::VectorXd &thetas_from, const Eigen::VectorXd &thetas_to) const;

        // to do
        Eigen::VectorXd IK(const Eigen::Matrix4d &pose_b_e) const { return Eigen::VectorXd::Ones(3); };

        manif::SE3d X_SE3[2];
        manif::SO3d X_SO3[2];
    };
    // enum HandeyeFlags
    // {
    //     HANDEYE_TYPE_AXXB = 0x00001,
    //     HANDEYE_TYPE_AXZB = 0x00002
    //     // CAMERA_K3P2_DIVIDE = 0x00002,
    //     // CAMERA_K6P2 = 0x00004,
    //     // CAMERA_FIX_K1 = 0x00008,
    //     // CAMERA_FIX_K2 = 0x00010,
    //     // CAMERA_FIX_K3 = 0x00020,
    //     // CAMERA_FIX_P = 0x00040,
    //     // CAMERA_FIX_K4 = 0x00080,
    //     // CAMERA_FIX_K5 = 0x00100,
    //     // CAMERA_FIX_K6 = 0x00200
    // };

    // inline HandeyeFlags operator|(HandeyeFlags a, HandeyeFlags b)
    // {
    //     return static_cast<HandeyeFlags>(static_cast<int>(a) | static_cast<int>(b));
    // }

    // struct KSfMOptions
    // {

    //     KSfMOptions() : maxIter(40),
    //                     rmseThreshold(1.0),
    //                     focalLength(1140),
    //                     newtonGaussDamping(0.25){};
    //     size_t maxIter;
    //     double rmseThreshold;
    //     double focalLength;
    //     double newtonGaussDamping;
    // };
    // double kba_X1X2L(const std::vector<Eigen::Matrix2Xd> &corners_undist_normalized,
    //                  const std::vector<Eigen::Matrix3Xd> &p_obj_eigen,
    //                  const std::vector<Eigen::Vector2d> &thetas,
    //                  Eigen::Matrix4d &X1, Eigen::Matrix4d &X2, Eigen::Matrix4d &L,
    //                  KSfMOptions option = KSfMOptions());

    // Eigen::Matrix2Xd createCheckerboardPoints(const Eigen::Vector2i &checkerboardSize, double length = 1.0);

    // double calibrateCamera(
    //     const std::vector<Eigen::Matrix2Xd> &views_image_p2ds,
    //     const std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
    //     Eigen::Matrix3d &K, Eigen::VectorXd &D, CalibFlags flags = CalibFlags::CAMERA_K3P2_MULTIPLY);
}

#endif