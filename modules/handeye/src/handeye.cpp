
#include "manif/manif.h"
#include "handeye/handeye.hpp"
#include "handeye/handeye_sfm.hpp"
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

            dXL = JTJ.fullPivHouseholderQr().solve(JTe);
            // std::cout << "res" << JTe << "\n";
            rmse = sqrt(res / n_data / n_objPoints) * option.focalLength;
            // JTe = JTe / n_data / n_objPoints;

            std::cout << "# RMSE ################################## " << std::sqrt(res / (double)count) * option.focalLength << std::endl;

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
