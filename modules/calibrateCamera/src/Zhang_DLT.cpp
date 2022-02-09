#include "calibrateCamera/Zhang_DLT.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace CalibrateCamera
{
    void vpq(const Eigen::Matrix3d &H, const int p, const int q, Eigen::VectorXd &vH)
    {
        vH.resize(6);
        vH(0) = H(0, p) * H(0, q);
        vH(1) = H(0, p) * H(1, q) + H(1, p) * H(0, q);
        vH(2) = H(1, p) * H(1, q);
        vH(3) = H(2, p) * H(0, q) + H(0, p) * H(2, q);
        vH(4) = H(2, p) * H(1, q) + H(1, p) * H(2, q);
        vH(5) = H(2, p) * H(2, q);
    }

    void calcNormalizationMatrix(const Eigen::Matrix2Xd &p2ds, Eigen::Matrix3d &N)
    {
        double accmx = 0, accmy = 0;
        Eigen::Vector2d p_avg = p2ds.rowwise().mean();
        for (Eigen::Index col = 0; col < p2ds.cols(); col++)
        {
            accmx += (p2ds(0, col) - p_avg.x()) * (p2ds(0, col) - p_avg.x());
            accmy += (p2ds(1, col) - p_avg.y()) * (p2ds(1, col) - p_avg.y());
        }
        double stdx = std::sqrt(accmx / double(p2ds.cols() - 1.0));
        double stdy = std::sqrt(accmy / double(p2ds.cols() - 1.0));
        double sx = std::sqrt(2.) / stdx;
        double sy = std::sqrt(2.) / stdy;
        N << sx, 0, -sx * p_avg.x(), 0, sy, -sy * p_avg.y(), 0, 0, 1;
    };

    void estimatePlanarHomographyMatrix(const Eigen::Matrix2Xd &image_p2ds,
                                        const Eigen::Matrix2Xd &board_p2ds,
                                        Eigen::Matrix3d &H)
    {
        assert(image_p2ds.cols() == board_p2ds.cols());
        Eigen::Matrix3d N_image;
        Eigen::Matrix3d N_board;
        int n = image_p2ds.cols();
        calcNormalizationMatrix(image_p2ds, N_image);
        calcNormalizationMatrix(board_p2ds, N_board);

        Eigen::MatrixXd M;
        M.resize(2 * n, 9);
        M.setZero();

        for (int i = 0; i < n; ++i)
        {
            Eigen::Vector3d norm_img_p = N_image * Eigen::Vector3d(image_p2ds(0, i), image_p2ds(1, i), 1);
            Eigen::Vector3d norm_board_p = N_board * Eigen::Vector3d(board_p2ds(0, i), board_p2ds(1, i), 1);
            // M
            M(2 * i, 0) = -norm_board_p(0);
            M(2 * i, 1) = -norm_board_p(1);
            M(2 * i, 2) = -1;
            M(2 * i, 6) = norm_img_p(0) * norm_board_p(0);
            M(2 * i, 7) = norm_img_p(0) * norm_board_p(1);
            M(2 * i, 8) = norm_img_p(0);
            M(2 * i + 1, 3) = -norm_board_p(0);
            M(2 * i + 1, 4) = -norm_board_p(1);
            M(2 * i + 1, 5) = -1;
            M(2 * i + 1, 6) = norm_img_p(1) * norm_board_p(0);
            M(2 * i + 1, 7) = norm_img_p(1) * norm_board_p(1);
            M(2 * i + 1, 8) = norm_img_p(1);
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
        Eigen::VectorXd V = svd.matrixV().col(8);
        H << V(0), V(1), V(2), V(3), V(4), V(5), V(6), V(7), V(8);
        H = N_image.inverse() * H * N_board;
        H /= H(2, 2);
    };

    void estimateIntrinsicMatrix(const std::vector<Eigen::Matrix3d> &H_mats,
                                 Eigen::Matrix3d &K)
    {
        int N = H_mats.size();
        Eigen::MatrixXd V(2 * N, 6);
        V.setZero();

        for (int n = 0; n < N; ++n)
        {
            Eigen::VectorXd v01, v00, v11;
            vpq(H_mats[n], 0, 1, v01);
            vpq(H_mats[n], 0, 0, v00);
            vpq(H_mats[n], 1, 1, v11);
            V.row(2 * n) = v01;
            V.row(2 * n + 1) = v00 - v11;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullV);
        Eigen::VectorXd b = svd.matrixV().col(5);
        Eigen::Matrix3d B;
        Eigen::Matrix3d L;
        B << b[0], b[1], b[3], b[1], b[2], b[4], b[3], b[4], b[5];
        if (B(0, 0) > 0 && B(1, 1) > 0 && B(2, 2) > 0)
        {
            L = (B.llt().matrixL());
        }
        else if (B(0, 0) < 0 && B(1, 1) < 0 && B(2, 2) < 0)
        {
            L = ((-1. * B).llt().matrixL());
        }
        else
        {
            printf("[WARNING] BAD HOMOGRAPHY MATRICES!\n");
        }
        K = L.transpose().inverse() * L(2, 2);
    };

    void calcExtrinsics(const std::vector<Eigen::Matrix3d> &H_mats,
                        const Eigen::Matrix3d &K,
                        std::vector<Eigen::Matrix4d> &Rt)
    {
        Rt.resize(H_mats.size(), Eigen::Matrix4d::Identity());
        Eigen::Matrix3d Kinv = K.inverse();
        for (size_t i = 0; i < H_mats.size(); ++i)
        {
            Eigen::Vector3d Ainvxh0 = Kinv * H_mats[i].col(0);
            Eigen::Vector3d Ainvxh1 = Kinv * H_mats[i].col(1);
            Eigen::Vector3d Ainvxh2 = Kinv * H_mats[i].col(2);
            double lambda = 1.0 / Ainvxh0.norm();
            Eigen::Vector3d r0 = lambda * Ainvxh0;
            Eigen::Vector3d r1 = lambda * Ainvxh1;
            Eigen::Vector3d r2 = r0.cross(r1);
            Eigen::Vector3d t = lambda * Ainvxh2;

            Rt[i].block<3, 1>(0, 0) = r0;
            Rt[i].block<3, 1>(0, 1) = r1;
            Rt[i].block<3, 1>(0, 2) = r2;
            Rt[i].block<3, 1>(0, 3) = t;
        }
    };

    // double calibrateCamera(
    //     const std::vector<Eigen::MatrixX2d> &views_image_p2ds,
    //     const std::vector<Eigen::MatrixX2d> &views_board_p2ds,
    //     Eigen::Matrix3d &K)

}