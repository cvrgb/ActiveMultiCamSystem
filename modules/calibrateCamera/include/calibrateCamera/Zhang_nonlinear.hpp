#ifndef CALIBRATECAMERA_ZHANG_NONLINEAR_HPP
#define CALIBRATECAMERA_ZHANG_NONLINEAR_HPP

#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <glog/logging.h>
#include "Zhang.hpp"
typedef Eigen::NumTraits<double> EigenDouble;
typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 8> MatX8;
typedef Eigen::Vector3d Vec3;

using namespace calibration_toolkit;

namespace CalibrateCamera
{
    struct RefineHomographyOptions
    {
        // Default settings for homography estimation which should be suitable
        // for a wide range of use cases.
        RefineHomographyOptions()
            : max_num_iterations(250), expected_average_symmetric_distance(1e-16) {}

        // Maximal number of iterations for the refinement step.
        int max_num_iterations;

        // Expected average of symmetric geometric distance between
        // actual destination points and original ones transformed by
        // estimated homography matrix.
        //
        // Refinement will finish as soon as average of symmetric
        // geometric distance is less or equal to this value.
        //
        // This distance is measured in the same units as input points are.
        double expected_average_symmetric_distance;
    };

    template <typename T>
    void SymmetricDistanceComponents(const Eigen::Matrix<T, 3, 3> &H,
                                     const Eigen::Matrix<T, 2, 1> &x1,
                                     const Eigen::Matrix<T, 2, 1> &x2,
                                     T forward_error[2],
                                     T backward_error[2])
    {
        typedef Eigen::Matrix<T, 3, 1> Vec3;
        Vec3 x(x1(0), x1(1), T(1.0));
        Vec3 y(x2(0), x2(1), T(1.0));

        Vec3 H_x = H * x;
        Vec3 Hinv_y = H.inverse() * y;

        H_x /= H_x(2);
        Hinv_y /= Hinv_y(2);

        forward_error[0] = 1.0 * (H_x(0) - y(0));
        forward_error[1] = 1.0 * (H_x(1) - y(1));
        backward_error[0] = 1.0 * (Hinv_y(0) - x(0));
        backward_error[1] = 1.0 * (Hinv_y(1) - x(1));
    }

    // Calculate symmetric geometric cost:
    //
    //   D(H * x1, x2)^2 + D(H^-1 * x2, x1)^2
    //
    double SymmetricDistance(const Mat3 &H,
                             const Vec2 &x1,
                             const Vec2 &x2);

    template <typename T = double>
    class Homography2DNormalizedParameterization
    {
    public:
        typedef Eigen::Matrix<T, 8, 1> Parameters;    // a, b, ... g, h
        typedef Eigen::Matrix<T, 3, 3> Parameterized; // H

        // Convert from the 8 parameters to a H matrix.
        static void To(const Parameters &p, Parameterized *h)
        {
            // clang-format off
    *h << p(0), p(1), p(2),
          p(3), p(4), p(5),
          p(6), p(7), 1.0;
            // clang-format on
        }

        // Convert from a H matrix to the 8 parameters.
        static void From(const Parameterized &h, Parameters *p)
        {
            // clang-format off
    *p << h(0, 0), h(0, 1), h(0, 2),
          h(1, 0), h(1, 1), h(1, 2),
          h(2, 0), h(2, 1);
            // clang-format on
        }
    };

    /// Cost functor which computes symmetric geometric distance
    // used for homography matrix refinement.
    class HomographySymmetricGeometricCostFunctor
    {
    public:
        HomographySymmetricGeometricCostFunctor(const Vec2 &x, const Vec2 &y)
            : x_(x), y_(y) {}

        template <typename T>
        bool operator()(const T *homography_parameters, T *residuals) const
        {
            typedef Eigen::Matrix<T, 3, 3> Mat3;
            typedef Eigen::Matrix<T, 2, 1> Vec2;

            Mat3 H(homography_parameters);
            Vec2 x(T(x_(0)), T(x_(1)));
            Vec2 y(T(y_(0)), T(y_(1)));

            SymmetricDistanceComponents<T>(H, x, y, &residuals[0], &residuals[2]);
            return true;
        }

        const Vec2 x_;
        const Vec2 y_;
    };

    // Termination checking callback. This is needed to finish the
    // optimization when an absolute error threshold is met, as opposed
    // to Ceres's function_tolerance, which provides for finishing when
    // successful steps reduce the cost function by a fractional amount.
    // In this case, the callback checks for the absolute average reprojection
    // error and terminates when it's below a threshold (for example all
    // points < 0.5px error).
    class TerminationCheckingCallback : public ceres::IterationCallback
    {
    public:
        TerminationCheckingCallback(const Mat &x1,
                                    const Mat &x2,
                                    const RefineHomographyOptions &options,
                                    Mat3 *H)
            : options_(options), x1_(x1), x2_(x2), H_(H) {}

        virtual ceres::CallbackReturnType operator()(
            const ceres::IterationSummary &summary)
        {
            // If the step wasn't successful, there's nothing to do.
            if (!summary.step_is_successful)
            {
                return ceres::SOLVER_CONTINUE;
            }

            // Calculate average of symmetric geometric distance.
            double average_distance = 0.0;
            for (int i = 0; i < x1_.cols(); i++)
            {
                average_distance += SymmetricDistance(*H_, x1_.col(i), x2_.col(i));
            }
            average_distance /= x1_.cols();

            if (average_distance <= options_.expected_average_symmetric_distance)
            {
                return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
            }

            return ceres::SOLVER_CONTINUE;
        }

    private:
        const RefineHomographyOptions &options_;
        const Mat &x1_;
        const Mat &x2_;
        Mat3 *H_;
    };

    bool RefineHomography(const Mat &x1, const Mat &x2,
                          const RefineHomographyOptions &options,
                          Mat3 *H);

    struct ReprojectionError_K3P2
    {
        ReprojectionError_K3P2(const Eigen::Vector2d &img_pts_, const Eigen::Vector2d &board_pts_,
                               const Eigen::VectorXd &k_init_,
                               calibration_toolkit::CalibFlags flag_ = calibration_toolkit::CAMERA_K3P2_MULTIPLY)
            : _img_pts(img_pts_), _board_pts(board_pts_), _flags(flag_), _k_init(k_init_)
        {
        }
        template <typename T>
        bool operator()(const T *const instrinsics_, const T *const k_, const T *const rt_, // 6 : angle axis and translation
                        T *residuls) const
        {
            // Eigen::Vector3d hom_w(_board_pts(0), _board_pts(1), T(1.));
            T hom_w_t[3];
            hom_w_t[0] = T(_board_pts(0));
            hom_w_t[1] = T(_board_pts(1));
            hom_w_t[2] = T(1.);
            T hom_w_trans[3];
            ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
            hom_w_trans[0] += rt_[3];
            hom_w_trans[1] += rt_[4];
            hom_w_trans[2] += rt_[5];

            T c_x = hom_w_trans[0] / hom_w_trans[2];
            T c_y = hom_w_trans[1] / hom_w_trans[2];

            // distortion
            T r2 = c_x * c_x + c_y * c_y;
            T r4 = r2 * r2;
            T r6 = r4 * r2;

            T k1_term = _flags & CalibFlags::CAMERA_FIX_K1 ? (T)_k_init[0] : k_[0];
            T k2_term = _flags & CalibFlags::CAMERA_FIX_K2 ? (T)_k_init[1] : k_[1];
            T k3_term = _flags & CalibFlags::CAMERA_FIX_K3 ? (T)_k_init[4] : k_[4];
            T p1_term = _flags & CalibFlags::CAMERA_FIX_P ? (T)_k_init[2] : k_[2];
            T p2_term = _flags & CalibFlags::CAMERA_FIX_P ? (T)_k_init[3] : k_[3];

            T r_coeff = T(1.0);
            if (_flags & CalibFlags::CAMERA_K3P2_DIVIDE)
                r_coeff = r_coeff / (T(1) + r2 * k1_term + r4 * k2_term + r6 * k3_term);
            else if (_flags & CalibFlags::CAMERA_K3P2_MULTIPLY)
                r_coeff = r_coeff * (T(1) + r2 * k1_term + r4 * k2_term + r6 * k3_term);
            else
                r_coeff = r_coeff * (T(1) + r2 * k1_term + r4 * k2_term + r6 * k3_term);
            // T r_coeff = T(1) * (T(1) + k_[0] * r2 + k_[1] * r4 + k_[4] * r6) / (T(1) + k_[0] * r2 + k_[1] * r4 + k_[4] * r6);
            T xd = c_x * r_coeff + 2.0 * p1_term * c_x * c_y + p2_term * (r2 + 2.0 * c_x * c_x);
            T yd = c_y * r_coeff + 2.0 * p2_term * c_x * c_y + p1_term * (r2 + 2.0 * c_y * c_y);

            T predict_x = instrinsics_[0] * xd + instrinsics_[2];
            T predict_y = instrinsics_[3] * yd + instrinsics_[4];
            // residus
            residuls[0] = _img_pts(0) - predict_x;
            residuls[1] = _img_pts(1) - predict_y;

            return true;
        }
        const Eigen::Vector2d _img_pts;
        const Eigen::Vector2d _board_pts;
        const Eigen::VectorXd _k_init;
        calibration_toolkit::CalibFlags _flags;
    };

    struct ReprojectionError_K6P2
    {
        ReprojectionError_K6P2(const Eigen::Vector2d &img_pts_, const Eigen::Vector2d &board_pts_, const Eigen::VectorXd &k_init_,
                               calibration_toolkit::CalibFlags flag_ = calibration_toolkit::CAMERA_K6P2)
            : _img_pts(img_pts_), _board_pts(board_pts_), _flags(flag_)
        {
        }
        template <typename T>
        bool operator()(const T *const instrinsics_, const T *const k_, const T *const rt_, // 6 : angle axis and translation
                        T *residuls) const
        {
            // Eigen::Vector3d hom_w(_board_pts(0), _board_pts(1), T(1.));
            T hom_w_t[3];
            hom_w_t[0] = T(_board_pts(0));
            hom_w_t[1] = T(_board_pts(1));
            hom_w_t[2] = T(1.);
            T hom_w_trans[3];
            ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
            hom_w_trans[0] += rt_[3];
            hom_w_trans[1] += rt_[4];
            hom_w_trans[2] += rt_[5];

            T c_x = hom_w_trans[0] / hom_w_trans[2];
            T c_y = hom_w_trans[1] / hom_w_trans[2];

            // distortion
            T r2 = c_x * c_x + c_y * c_y;
            T r4 = r2 * r2;
            T r6 = r4 * r2;

            T k1_term = _flags & CalibFlags::CAMERA_FIX_K1 ? (T)_k_init[0] : k_[0];
            T k2_term = _flags & CalibFlags::CAMERA_FIX_K2 ? (T)_k_init[1] : k_[1];
            T k3_term = _flags & CalibFlags::CAMERA_FIX_K3 ? (T)_k_init[4] : k_[4];
            T p1_term = _flags & CalibFlags::CAMERA_FIX_P ? (T)_k_init[2] : k_[2];
            T p2_term = _flags & CalibFlags::CAMERA_FIX_P ? (T)_k_init[3] : k_[3];
            T k4_term = _flags & CalibFlags::CAMERA_FIX_K4 ? (T)_k_init[0] : k_[5];
            T k5_term = _flags & CalibFlags::CAMERA_FIX_K5 ? (T)_k_init[1] : k_[6];
            T k6_term = _flags & CalibFlags::CAMERA_FIX_K6 ? (T)_k_init[4] : k_[7];

            T r_coeff = T(1.0);
            r_coeff = r_coeff * (T(1) + r2 * k1_term + r4 * k2_term + r6 * k3_term) / (T(1) + r2 * k4_term + r4 * k5_term + r6 * k6_term);
            T xd = c_x * r_coeff + 2.0 * p1_term * c_x * c_y + p2_term * (r2 + 2.0 * c_x * c_x);
            T yd = c_y * r_coeff + 2.0 * p2_term * c_x * c_y + p1_term * (r2 + 2.0 * c_y * c_y);

            T predict_x = instrinsics_[0] * xd + instrinsics_[2];
            T predict_y = instrinsics_[3] * yd + instrinsics_[4];
            // residus
            residuls[0] = _img_pts(0) - predict_x;
            residuls[1] = _img_pts(1) - predict_y;

            return true;
        }
        const Eigen::Vector2d _img_pts;
        const Eigen::Vector2d _board_pts;
        calibration_toolkit::CalibFlags _flags;
        const Eigen::VectorXd _k_init;
    };

    double refineCalibration(
        const std::vector<Eigen::Matrix2Xd> &views_image_p2ds,
        const std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
        const std::vector<Eigen::Matrix4d> &Rt,
        Eigen::Matrix3d &K, Eigen::VectorXd &D,
        calibration_toolkit::CalibFlags flags);

}

#endif