
#include "manif/manif.h"
#include "handeye/handeye.hpp"
#include "handeye/handeye_sfm.hpp"

#include <Eigen/Dense>
namespace calibration_toolkit
{

    double kba_X1X2L(const std::vector<Eigen::Matrix2Xd> &corners_undist_normalized,
                     const std::vector<Eigen::Matrix3Xd> &p_obj_eigen,
                     const std::vector<Eigen::Vector2d> &thetas,

                     Eigen::Matrix4d &X1_, Eigen::Matrix4d &X2_, Eigen::Matrix4d &L_,
                     KSfMOptions option)
    {

        // const size_t maxIter = 40;
        Eigen::VectorXd JTe, dXL;
        Eigen::MatrixXd JTJ;
        JTJ.resize(18, 18);
        JTe.resize(18);
        dXL.resize(18);
        double res = 1000;
        double rmse = 1000.0;

        // Eigen::Isometry3d();

        manif::SE3d X1(X1_.block<3, 1>(0, 3), Eigen::AngleAxisd(X1_.block<3, 3>(0, 0)));
        manif::SE3d X2(X2_.block<3, 1>(0, 3), Eigen::AngleAxisd(X2_.block<3, 3>(0, 0)));
        manif::SE3d L(L_.block<3, 1>(0, 3), Eigen::AngleAxisd(L_.block<3, 3>(0, 0)));

        double t1 = 0.5;
        manif::SE3d testB1 = Bm(X1, t1);
        double t2 = 0.1;
        manif::SE3d testB2 = Bm(X2, t2);
        Eigen::Matrix<double, 6, 6> testJB = testB2.adj().inverse() * J66_Bm_Xm(testB1);

        // Eigen::Vector3d ttt = X1.rotation().transpose() * Eigen::Vector3d(0, 0, 1);
        // null0(0 + 3) = ttt.x();
        // null0(0 + 4) = ttt.y();
        // null0(0 + 5) = ttt.z();
        // std::cout << (testJB * null0).transpose() << "\n";

        int my_int;
        std::cin >> my_int;

        size_t n_data = corners_undist_normalized.size();
        size_t n_objPoints = p_obj_eigen.size();

        size_t iter = 0;

        std::cout << X1.transform() << std::endl;
        std::cout << X2.transform() << std::endl;
        std::cout << L.transform() << std::endl;

        bool another = false;
        while (iter < option.maxIter)
        {
            JTe.setZero();
            dXL.setZero();
            JTJ.setZero();
            res = 0;
            int count = 0;

            Eigen::VectorXd eei;
            Eigen::MatrixXd JJi;

            for (size_t i_frame = 0; i_frame < corners_undist_normalized.size(); i_frame++)
            {
                reprojection(corners_undist_normalized[i_frame], p_obj_eigen[i_frame],
                             X1, X2, thetas[i_frame].x(), thetas[i_frame].y(), L, eei, JJi);
                // if (another)
                // {
                // std::cout << "# LOCAL RMS ################################## " << eei.squaredNorm() << std::endl;
                JTe += JJi.transpose() * eei;

                JTJ += JJi.transpose() * JJi;
                res += eei.squaredNorm();
                count = count + corners_undist_normalized[i_frame].cols(); //

                //}
            }

            double mu1 = 1.0;
            double nu1 = 1.0;
            double mu2 = 1.0;
            double nu2 = 1.0;

            Eigen::Vector3d null1omega = mu1 * X1.rotation().transpose() * Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d null1rho = mu1 * X1.rotation().transpose() * manif::SO3Tangentd(Eigen::Vector3d(0, 0, 1)).hat() * X1.translation() + nu1 * X1.rotation().transpose() * Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d null2omega = mu2 * X2.rotation().transpose() * Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d null2rho = mu2 * X2.rotation().transpose() * manif::SO3Tangentd(Eigen::Vector3d(0, 0, 1)).hat() * X2.translation() + nu2 * X2.rotation().transpose() * Eigen::Vector3d(0, 0, 1);
            Eigen::VectorXd null0;
            null0.resize(18);
            null0.setZero();
            null0(0 + 0) = null1rho.x();
            null0(0 + 1) = null1rho.y();
            null0(0 + 2) = null1rho.z();
            null0(0 + 3) = null1omega.x();
            null0(0 + 4) = null1omega.y();
            null0(0 + 5) = null1omega.z();
            null0(6 + 0) = null2rho.x();
            null0(6 + 1) = null2rho.y();
            null0(6 + 2) = null2rho.z();
            null0(6 + 3) = null2omega.x();
            null0(6 + 4) = null2omega.y();
            null0(6 + 5) = null2omega.z();

            std::cout << (JTJ * null0).transpose() << "\n";

            Eigen::MatrixXd lambda = JTJ;
            lambda.setIdentity();
            for (int di = 0; di < JTJ.rows(); di++)
            {
                lambda(di, di) = JTJ(di, di);
            }
            // lambda = lambda;
            // JTJ += lambda;

            dXL = JTJ.fullPivHouseholderQr().solve(JTe);
            // std::cout << "res" << JTe << "\n";
            rmse = sqrt(res / n_data / n_objPoints) * option.focalLength;
            // JTe = JTe / n_data / n_objPoints;

            std::cout << "# RMSE ################################## " << std::sqrt(res / (double)count) * option.focalLength << std::endl;

            manif::SE3d T = X1 * X2.inverse();
            Eigen::Matrix3d R = T.rotation();
            Eigen::Vector3d t = T.translation();
            Eigen::Vector3d zhat(0, 0, 1);
            Eigen::Vector3d a0 = t;
            Eigen::Vector3d b0 = zhat.cross(R * zhat);
            std::cout << abs(a0.dot(b0) / b0.dot(b0)) << std::endl;

            // A << 1, 2, 2, 3;
            // cout << "Here is the matrix A:\n"
            //      << A << endl;

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JTJ);
            if (eigensolver.info() != Eigen::Success)
                abort();
            std::cout << "The eigenvalues of A are:\n"
                      << eigensolver.eigenvalues() << std::endl;
            // cout << "Here's a matrix whose columns are eigenvectors of A \n"
            //      << "corresponding to these eigenvalues:\n"
            //      << eigensolver.eigenvectors() << endl;

            // double damp_rho = 1.0;
            manif::SE3Tangentd x1 = manif::SE3Tangentd(-option.newtonGaussDamping * (Eigen::Matrix<double, 6, 1>() << dXL(0), dXL(1), dXL(2), dXL(3), dXL(4), dXL(5)).finished());
            manif::SE3Tangentd x2 = manif::SE3Tangentd(-option.newtonGaussDamping * (Eigen::Matrix<double, 6, 1>() << dXL(6), dXL(7), dXL(8), dXL(9), dXL(10), dXL(11)).finished());
            manif::SE3Tangentd l = manif::SE3Tangentd(-option.newtonGaussDamping * (Eigen::Matrix<double, 6, 1>() << dXL(12), dXL(13), dXL(14), dXL(15), dXL(16), dXL(17)).finished());
            X1 = X1 * x1.exp();
            X2 = X2 * x2.exp();
            L = L * l.exp();
            if (std::sqrt(res / (double)count) * option.focalLength < 3.0)
                another = true;
            iter++;
        }
        Eigen::IOFormat ExportFmt(Eigen::FullPrecision, 0, ", ", ",\n", "", "", "", "");
        std::cout << (X1 * X2.inverse()).transform().format(ExportFmt) << std::endl;

        std::cout << X1.transform().inverse() << std::endl;
        std::cout << X2.transform().inverse() << std::endl;
        std::cout << L.transform() << std::endl;

        return rmse;

        //
        // std::cout << "# RESULTS ###############################################################################" << std::endl;
        // std::cout << X1_tmp.transform().format(HeavyFmt) << std::endl;
        // std::cout << X2_tmp.transform().format(HeavyFmt) << std::endl;
        // std::cout << L_tmp.transform().format(HeavyFmt) << std::endl;
        // std::cout << X1_tmp.inverse().transform().format(HeavyFmt) << std::endl;
        // std::cout << X2_tmp.inverse().transform().format(HeavyFmt) << std::endl;
        // std::cout << L_tmp.inverse().transform().format(HeavyFmt) << std::endl;
    };

} // namespace calibration_toolkit
