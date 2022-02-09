#include "calibrateCamera/Zhang.hpp"
#include "calibrateCamera/Zhang_DLT.hpp"
#include "calibrateCamera/Zhang_nonlinear.hpp"

using namespace CalibrateCamera;

namespace calibration_toolkit
{
    Eigen::Matrix2Xd createCheckerboardPoints(const Eigen::Vector2i &checkerboardSize, double length)
    {

        // std::vector<cv::Point2f> vec_points;

        Eigen::Matrix2Xd board_p2ds;
        board_p2ds.resize(2, checkerboardSize.x() * checkerboardSize.y());
        board_p2ds.setZero();

        int counter = 0;
        for (int r = 0; r < checkerboardSize.y(); ++r)
        {
            for (int c = 0; c < checkerboardSize.x(); ++c)
            {
                board_p2ds(0, counter) = c * length;
                board_p2ds(1, counter) = r * length;
                counter++;
                // vec_points.emplace_back(c, r);
            }
        }
        // std::cout << board_p2ds << std::endl;
        //_boards_pts.push_back(vec_points);
        return board_p2ds;
    };

    double calibrateCamera(
        const std::vector<Eigen::Matrix2Xd> &views_image_p2ds,
        const std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
        Eigen::Matrix3d &K,
        Eigen::VectorXd &D,
        CalibFlags flags)

    {

        std::vector<Eigen::Matrix3d> H_mats;
        size_t n_views = views_board_p2ds.size();

        H_mats.resize(n_views, Eigen::Matrix3d::Identity());
        RefineHomographyOptions options;
        options.expected_average_symmetric_distance = 0.02;
        for (size_t iter_view = 0; iter_view < n_views; iter_view++)
        {
            estimatePlanarHomographyMatrix(views_image_p2ds[iter_view], views_board_p2ds[iter_view], H_mats[iter_view]);

            Eigen::MatrixXd x0 = views_board_p2ds[iter_view];
            Eigen::MatrixXd x1 = views_image_p2ds[iter_view];
            CalibrateCamera::RefineHomography(x0, x1, options, &H_mats[iter_view]);
        }

        estimateIntrinsicMatrix(H_mats, K);
        std::vector<Eigen::Matrix4d> Rt_mats;
        calcExtrinsics(H_mats, K, Rt_mats);
        Eigen::VectorXd D_internal;
        if ((flags & calibration_toolkit::CAMERA_K3P2_DIVIDE) || (flags & calibration_toolkit::CAMERA_K3P2_MULTIPLY))
            D_internal.resize(5);
        else if (flags & calibration_toolkit::CAMERA_K6P2)
            D_internal.resize(8);
        else
            D_internal.resize(5);
        D_internal.setZero();

        // std::cout << K << std::endl;
        double rmse = refineCalibration(views_image_p2ds, views_board_p2ds, Rt_mats, K, D_internal, flags);

        if (flags & calibration_toolkit::CAMERA_K3P2_MULTIPLY)
        {
            D.resize(5);
            D = D_internal;
        }
        else if (flags & calibration_toolkit::CAMERA_K3P2_DIVIDE)
        {
            D.resize(8);
            D(0) = 0.0;
            D(1) = 0.0;
            D(2) = 0.0;
            D(3) = D_internal(3);
            D(4) = D_internal(4);
            D(5) = D_internal(0);
            D(6) = D_internal(1);
            D(7) = D_internal(2);
        }
        else if (flags & calibration_toolkit::CAMERA_K6P2)
        {
            D.resize(8);
            D = D_internal;
        }

        return rmse;
    };
}
