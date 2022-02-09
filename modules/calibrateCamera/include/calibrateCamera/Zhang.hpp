#ifndef CALIBRATECAMERA_ZHANG_HPP
#define CALIBRATECAMERA_ZHANG_HPP
#include <vector>
#include <Eigen/Core>

namespace calibration_toolkit

{
    enum CalibFlags
    {
        CAMERA_K3P2_MULTIPLY = 0x00001,
        CAMERA_K3P2_DIVIDE = 0x00002,
        CAMERA_K6P2 = 0x00004,
        CAMERA_FIX_K1 = 0x00008,
        CAMERA_FIX_K2 = 0x00010,
        CAMERA_FIX_K3 = 0x00020,
        CAMERA_FIX_P = 0x00040,
        CAMERA_FIX_K4 = 0x00080,
        CAMERA_FIX_K5 = 0x00100,
        CAMERA_FIX_K6 = 0x00200
    };

    inline CalibFlags operator|(CalibFlags a, CalibFlags b)
    {
        return static_cast<CalibFlags>(static_cast<int>(a) | static_cast<int>(b));
    }

    Eigen::Matrix2Xd createCheckerboardPoints(const Eigen::Vector2i &checkerboardSize, double length = 1.0);

    double calibrateCamera(
        const std::vector<Eigen::Matrix2Xd> &views_image_p2ds,
        const std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
        Eigen::Matrix3d &K, Eigen::VectorXd &D, CalibFlags flags = CalibFlags::CAMERA_K3P2_MULTIPLY);

}

#endif