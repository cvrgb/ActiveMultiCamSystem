#ifndef HANDEYE_HPP
#define HANDEYE_HPP
#include <vector>
#include <Eigen/Core>

namespace calibration_toolkit
{
    enum HandeyeFlags
    {
        HANDEYE_TYPE_AXXB = 0x00001,
        HANDEYE_TYPE_AXZB = 0x00002
        // CAMERA_K3P2_DIVIDE = 0x00002,
        // CAMERA_K6P2 = 0x00004,
        // CAMERA_FIX_K1 = 0x00008,
        // CAMERA_FIX_K2 = 0x00010,
        // CAMERA_FIX_K3 = 0x00020,
        // CAMERA_FIX_P = 0x00040,
        // CAMERA_FIX_K4 = 0x00080,
        // CAMERA_FIX_K5 = 0x00100,
        // CAMERA_FIX_K6 = 0x00200
    };

    inline HandeyeFlags operator|(HandeyeFlags a, HandeyeFlags b)
    {
        return static_cast<HandeyeFlags>(static_cast<int>(a) | static_cast<int>(b));
    }

    struct KSfMOptions
    {

        KSfMOptions() : maxIter(40),
                        rmseThreshold(1.0),
                        focalLength(1140),
                        newtonGaussDamping(0.25){};
        size_t maxIter;
        double rmseThreshold;
        double focalLength;
        double newtonGaussDamping;
    };
    double kba_X1X2L(const std::vector<Eigen::Matrix2Xd> &corners_undist_normalized,
                     const std::vector<Eigen::Matrix3Xd> &p_obj_eigen,
                     const std::vector<Eigen::Vector2d> &thetas,
                     Eigen::Matrix4d &X1, Eigen::Matrix4d &X2, Eigen::Matrix4d &L,
                     KSfMOptions option = KSfMOptions());

    // Eigen::Matrix2Xd createCheckerboardPoints(const Eigen::Vector2i &checkerboardSize, double length = 1.0);

    // double calibrateCamera(
    //     const std::vector<Eigen::Matrix2Xd> &views_image_p2ds,
    //     const std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
    //     Eigen::Matrix3d &K, Eigen::VectorXd &D, CalibFlags flags = CalibFlags::CAMERA_K3P2_MULTIPLY);
}

#endif