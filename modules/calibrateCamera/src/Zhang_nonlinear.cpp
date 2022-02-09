

#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include "calibrateCamera/Zhang_nonlinear.hpp"
using namespace calibration_toolkit;

namespace CalibrateCamera
{

    // Calculate symmetric geometric cost:
    //   D(H * x1, x2)^2 + D(H^-1 * x2, x1)^2
    //
    double SymmetricDistance(const Mat3 &H,
                             const Vec2 &x1,
                             const Vec2 &x2)
    {

        Vec2 forward_error, backward_error;
        SymmetricDistanceComponents<double>(
            H, x1, x2, forward_error.data(), backward_error.data());
        return forward_error.squaredNorm() + backward_error.squaredNorm();
    }

    bool RefineHomography(
        const Mat &x1,
        const Mat &x2,
        const RefineHomographyOptions &options,
        Mat3 *H)
    {
        assert(2 == x1.rows());
        assert(4 <= x1.cols());
        assert(x1.rows() == x2.rows());
        assert(x1.cols() == x2.cols());

        ceres::Problem problem;
        for (int i = 0; i < x1.cols(); i++)
        {
            HomographySymmetricGeometricCostFunctor *
                homography_symmetric_geometric_cost_function =
                    new HomographySymmetricGeometricCostFunctor(x1.col(i), x2.col(i));

            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<HomographySymmetricGeometricCostFunctor,
                                                4, // num_residuals
                                                9>(
                    homography_symmetric_geometric_cost_function),
                nullptr,
                H->data());
        }

        // Configure the solve.
        ceres::Solver::Options solver_options;
        solver_options.linear_solver_type = ceres::DENSE_QR;
        solver_options.max_num_iterations = options.max_num_iterations;
        solver_options.update_state_every_iteration = true;

        // Terminate if the average symmetric distance is good enough.
        TerminationCheckingCallback callback(x1, x2, options, H);
        solver_options.callbacks.push_back(&callback);

        // Run the solve.
        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);

        // LOG(INFO) << "Summary:\n"
        //           << summary.FullReport();
        *H = *H / (*H)(2, 2);
        // LOG(INFO) << "Final refined matrix:\n"
        //           << *H;

        return summary.IsSolutionUsable();
    }

    double refineCalibration(
        const std::vector<Eigen::Matrix2Xd> &views_image_p2ds, const std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
        const std::vector<Eigen::Matrix4d> &Rt, Eigen::Matrix3d &K, Eigen::VectorXd &D, CalibFlags flags)
    {

        size_t n_views = views_image_p2ds.size();
        Eigen::VectorXd K_vec(5);
        K(0, 1) = 0.0;
        K_vec << K(0, 0), K(0, 1), K(0, 2), K(1, 1), K(1, 2);
        Eigen::VectorXd D_vec = D;
        ceres::Problem problem;
        std::vector<Eigen::VectorXd> Rt_vec;

        int dist_model = 5;
        if ((flags & calibration_toolkit::CAMERA_K3P2_DIVIDE) || (flags & calibration_toolkit::CAMERA_K3P2_MULTIPLY))
            dist_model = 5;
        else if ((flags & calibration_toolkit::CAMERA_K6P2))
            dist_model = 8;

        assert(dist_model == D_vec.size());

        for (size_t n = 0; n < Rt.size(); ++n)
        {
            Eigen::AngleAxisd rAxis(Rt[n].block<3, 3>(0, 0));
            Eigen::VectorXd rot_vec(rAxis.axis() * rAxis.angle());
            Eigen::VectorXd rt(6);
            rt << rot_vec(0), rot_vec(1), rot_vec(2), Rt[n](0, 3), Rt[n](1, 3), Rt[n](2, 3);
            Rt_vec.push_back(rt);
        }

        double *p_camera = K_vec.data();
        double *p_dist = D_vec.data();
        // For each view

        double rms = 0.0;
        int n_rms = 0;

        for (size_t n = 0; n < n_views; ++n)
        {
            double *p_rt = &Rt_vec[n](0);
            size_t n_points = views_image_p2ds[n].cols();
            // For each point in the view
            for (size_t i = 0; i < n_points; i++)
            {

                if (dist_model == 5)
                {

                    ReprojectionError_K3P2 *cost_function =
                        new ReprojectionError_K3P2(views_image_p2ds[n].col(i), views_board_p2ds[n].col(i), D, flags);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<
                            ReprojectionError_K3P2,
                            2, // num_residuals
                            5, // num_intrinsics
                            5, // num_distortion
                            6  // num_extrinsics
                            >(cost_function),
                        NULL,
                        p_camera,
                        p_dist,
                        p_rt);
                }
                else if (dist_model == 8)
                {
                    ReprojectionError_K6P2 *cost_function =
                        new ReprojectionError_K6P2(views_image_p2ds[n].col(i), views_board_p2ds[n].col(i), D, flags);
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<
                            ReprojectionError_K6P2,
                            2, // num_residuals
                            5, // num_intrinsics
                            8, // num_distortion
                            6  // num_extrinsics
                            >(cost_function),
                        NULL,
                        p_camera,
                        p_dist,
                        p_rt);
                }

                n_rms++;
            }
        }
        // Configure the solver.
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = false;

        // Solve!
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        rms = summary.final_cost;

        // std::cout <<  << "\n";
        //  DLOG(INFO) << "Final Brief Report:\n"
        //             << summary.BriefReport() << std::endl;
        //  K_vec << K(0, 0), K(0, 1) * 0.0, K(0, 2), K(1, 1), K(1, 2);
        K(0, 0) = K_vec(0);
        K(0, 2) = K_vec(2);
        K(1, 1) = K_vec(3);
        K(1, 2) = K_vec(4);
        D = D_vec;

        return std::sqrt(rms / n_rms);

        // std::cout << K << "\n";
        // std::cout << D << "\n";
    }

}
