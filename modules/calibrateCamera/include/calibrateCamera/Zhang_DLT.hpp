#ifndef CALIBRATECAMERA_ZHANG_DLT_HPP
#define CALIBRATECAMERA_ZHANG_DLT_HPP
#include <vector>
#include <Eigen/Core>

namespace CalibrateCamera
{

    // void calcNormalizationMatrix(const Eigen::MatrixX2d &p2ds, Eigen::Matrix3d &N);
    void estimatePlanarHomographyMatrix(const Eigen::Matrix2Xd &image_p2ds,
                                        const Eigen::Matrix2Xd &board_p2ds,
                                        Eigen::Matrix3d &H);
    void estimateIntrinsicMatrix(const std::vector<Eigen::Matrix3d> &H_mats,
                                 Eigen::Matrix3d &K);
    void calcExtrinsics(const std::vector<Eigen::Matrix3d> &H_mats,
                        const Eigen::Matrix3d &K,
                        std::vector<Eigen::Matrix4d> &Rt);
}

#endif