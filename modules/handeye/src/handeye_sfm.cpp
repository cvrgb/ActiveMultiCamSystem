#include "handeye/handeye_sfm.hpp"
#include <iostream>

namespace calibration_toolkit
{

    Eigen::Matrix<double, 6, 6> J66_Bm_Xm(const manif::SE3d &Bm)
    {
        manif::SE3Tangentd bm = Bm.log();
        return bm.rjac() * bm.smallAdj();
    };

    Eigen::Matrix<double, 3, 6> J36_Ipx(const Eigen::Vector3d &p)
    {
        manif::SO3Tangentd p_so3 = p;
        Eigen::Matrix<double, 3, 6> out;
        out.block(0, 0, 3, 3).setIdentity();
        out.block(0, 3, 3, 3) = -1.0 * p_so3.hat();
        return out;
    };

    Eigen::Vector2d proj(const Eigen::Vector3d &p)
    {
        return Eigen::Vector2d(p.x() / p.z(), p.y() / p.z());
    }

    Eigen::Matrix<double, 6, 12> J612_B_X(const manif::SE3d &B1, const manif::SE3d &B2)
    {
        Eigen::Matrix<double, 6, 12> out;
        out.setZero();
        out.block(0, 0, 6, 6) = B2.adj().inverse() * J66_Bm_Xm(B1);
        out.block(0, 6, 6, 6) = J66_Bm_Xm(B2);
        return out;
    };

    Eigen::Matrix<double, 2, 3> J23_pi(const Eigen::Vector3d &p)
    {
        Eigen::Matrix<double, 2, 3> out;
        out.block(0, 0, 2, 2).setIdentity();
        out(0, 2) = -p.x() / p.z();
        out(1, 2) = -p.y() / p.z();
        out = out / p.z();
        return out;
    };

    void J36(const manif::SE3d &B, const manif::SE3d &L, const Eigen::Vector3d &p_obj,

             Eigen::Matrix<double, 3, 6> &J_B, Eigen::Matrix<double, 3, 6> &J_L)
    {
        Eigen::Matrix3d R = (B.inverse() * L).rotation();
        Eigen::Matrix<double, 3, 6> IPX = J36_Ipx(p_obj);
        J_B = -R * IPX * ((B.inverse() * L).adj().inverse());
        J_L = R * IPX;
    }
    manif::SE3d Bm(const manif::SE3d &Xm, double theta)
    {

        return Xm.inverse() * ((theta * a_se3).exp()) * Xm;
    }
    Eigen::Matrix4d B12(const manif::SE3d &X1, const manif::SE3d &X2, double theta1, double theta2)
    {
        return (Bm(X1, theta1) * Bm(X2, theta2)).transform();
    }

    void J218_p_XL(
        const manif::SE3d &B1, const manif::SE3d &B2, //, double theta1, double theta2,
        const manif::SE3d &L,
        const Eigen::Vector3d &p_obj,
        Eigen::Vector2d &p_obj_proj,
        Eigen::Matrix<double, 2, 12> &Jij_X,
        Eigen::Matrix<double, 2, 6> &Jij_L)
    {
        manif::SE3d B12 = B1 * B2;
        Eigen::Vector4d p4d_obj(p_obj.x(), p_obj.y(), p_obj.z(), 1);
        Eigen::Vector4d p4d_cam = (B12.inverse() * L).transform() * p4d_obj;
        Eigen::Vector3d p3d_cam(p4d_cam.x(), p4d_cam.y(), p4d_cam.z());
        Eigen::Matrix<double, 2, 3> J_pi = J23_pi(p3d_cam);
        Eigen::Matrix<double, 3, 6> J_B, J_L;
        Eigen::Matrix<double, 6, 12> J_B_X = J612_B_X(B1, B2);
        J36(B12, L, p_obj, J_B, J_L);

        p_obj_proj = proj(p3d_cam);
        Jij_L = J_pi * J_L;
        Jij_X = J_pi * J_B * J_B_X;
        // manif::SE3d B1 = Bm(X1, theta1);
        // manif::SE3d B2 = Bm(X2, theta2);
        // manif::SE3d B2 = Bm(X2, theta2);
    }

    void reprojection(const Eigen::Matrix2Xd &p_img, const Eigen::Matrix3Xd &p_obj,
                      const manif::SE3d &X1, const manif::SE3d &X2,
                      double theta1, double theta2, const manif::SE3d &L, Eigen::VectorXd &ei, Eigen::MatrixXd &Ji)
    {
        size_t N_pt_obj = p_obj.cols();

        // if (ei.size() != 2 * N_pt_obj)
        ei.resize(2 * N_pt_obj);
        Ji.resize(2 * N_pt_obj, 18);

        ei.setZero();
        Ji.setZero();

        manif::SE3d B1 = Bm(X1, theta1);
        manif::SE3d B2 = Bm(X2, theta2);
        Eigen::Matrix<double, 2, 12> Jij_X;
        Eigen::Matrix<double, 2, 6> Jij_L;
        // const manif::SE3d Bj = B1 * B2;

        for (size_t j = 0; j < N_pt_obj; j++)
        {
            Eigen::Vector2d pij_obj, pij_img;
            J218_p_XL(B1, B2, L, p_obj.col(j), pij_obj, Jij_X, Jij_L);
            pij_img = p_img.col(j);
            ei(2 * j) = pij_obj.x() - pij_img.x();
            ei(2 * j + 1) = pij_obj.y() - pij_img.y();
            Ji.block(2 * j, 0, 2, 12) = Jij_X;
            Ji.block(2 * j, 12, 2, 6) = Jij_L;
        }
    }
    /*
        double optimize_X1X2L(const std::vector<std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>> &corners_undist_normalized,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &p_obj_eigen,
                              const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &thetas,
                              manif::SE3d &X1_tmp, manif::SE3d &X2_tmp, manif::SE3d &L_tmp)
        {
            const size_t maxIter = 40;
            Eigen::VectorXd JTe, dXL;
            Eigen::MatrixXd JTJ;
            JTJ.resize(18, 18);
            JTe.resize(18);
            dXL.resize(18);
            double res = 1000;
            double rmse = 1000.0;
            double dampingGN = 0.25;
            double rmse_threshold = 1.0;
            double focal_length = 1140;

            size_t n_data = corners_undist_normalized.size();
            size_t n_objPoints = p_obj_eigen.size();

            size_t iter = 0;

            while (iter < maxIter && rmse > rmse_threshold)
            // for (size_t ggg = 0; ggg < maxIter; ggg++)
            {
                JTe.setZero();
                dXL.setZero();
                JTJ.setZero();
                res = 0;

                Eigen::VectorXd eei;
                Eigen::MatrixXd JJi;

                for (size_t i_frame = 0; i_frame < corners_undist_normalized.size(); i_frame++)
                {
                    abv_core::reprojection(corners_undist_normalized[i_frame], p_obj_eigen, X1_tmp, X2_tmp, thetas[i_frame].x(), thetas[i_frame].y(), L_tmp, eei, JJi);
                    JTe += JJi.transpose() * eei;
                    JTJ += JJi.transpose() * JJi;
                    res += eei.squaredNorm();
                }

                dXL = JTJ.fullPivHouseholderQr().solve(JTe);
                rmse = sqrt(res / n_data / n_objPoints) * focal_length;
    // JTe = JTe / n_data / n_objPoints;
    #ifdef MYDEBUG
                std::cout << "# RMSE ################# " << rmse << std::endl;
    #endif
                double damp_rho = 1.0;
                manif::SE3Tangentd x1 = manif::SE3Tangentd(-dampingGN * (Eigen::Matrix<double, 6, 1>() << damp_rho * dXL(0), damp_rho * dXL(1), damp_rho * dXL(2), dXL(3), dXL(4), dXL(5)).finished());
                manif::SE3Tangentd x2 = manif::SE3Tangentd(-dampingGN * (Eigen::Matrix<double, 6, 1>() << damp_rho * dXL(6), damp_rho * dXL(7), damp_rho * dXL(8), dXL(9), dXL(10), dXL(11)).finished());
                manif::SE3Tangentd l = manif::SE3Tangentd(-dampingGN * (Eigen::Matrix<double, 6, 1>() << dXL(12), dXL(13), dXL(14), dXL(15), dXL(16), dXL(17)).finished());
                X1_tmp = X1_tmp * x1.exp();
                X2_tmp = X2_tmp * x2.exp();
                L_tmp = L_tmp * l.exp();
                iter++;
            }

            return rmse;

            //
            // std::cout << "# RESULTS ###############################################################################" << std::endl;
            // std::cout << X1_tmp.transform().format(HeavyFmt) << std::endl;
            // std::cout << X2_tmp.transform().format(HeavyFmt) << std::endl;
            // std::cout << L_tmp.transform().format(HeavyFmt) << std::endl;
            // std::cout << X1_tmp.inverse().transform().format(HeavyFmt) << std::endl;
            // std::cout << X2_tmp.inverse().transform().format(HeavyFmt) << std::endl;
            // std::cout << L_tmp.inverse().transform().format(HeavyFmt) << std::endl;
        }
        */

} // namespace abv_core