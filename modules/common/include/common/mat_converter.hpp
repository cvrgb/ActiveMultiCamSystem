/*
 * ActiveBinocularVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Chengzhe Zhou
 * Copyright (C) 2021 ActiveBionicVision Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef COMMON_MAT_CONVERTER_HPP
#define COMMON_MAT_CONVERTER_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <typeinfo>
// #include <sophus/so3.hpp>
// #include <sophus/se3.hpp>

namespace calibration_toolkit
{

    // enum class MatConverterType // "enum class" defines this as a scoped enumeration instead of a standard enumeration
    // {
    //     id,
    //     timeStamp,
    //     motorData,
    //     imuData,
    //     jpg,
    //     png,
    //     bmp,
    //     ENCODER_RAD,
    //     ENCODER_DEG
    // };

    namespace MatConverter
    {
        // public:
        template <typename T1, typename T2>
        static std::vector<cv::Point_<T1>> cvPoint2(std::vector<cv::Point_<T2>> pt_T2)
        {
            std::vector<cv::Point_<T1>> pt_T1;
            pt_T1.resize(pt_T2.size());
            for (size_t k = 0; k < pt_T2.size(); k++)
                pt_T1[k] = cv::Point_<T1>(pt_T2[k].x, pt_T2[k].y);
            return pt_T1;
        };

        template <typename T1, typename T2>
        static std::vector<cv::Point3_<T1>> cvPoint3(std::vector<cv::Point3_<T2>> pt_T2)
        {
            std::vector<cv::Point3_<T1>> pt_T1;
            pt_T1.resize(pt_T2.size());
            for (size_t k = 0; k < pt_T2.size(); k++)
                pt_T1[k] = cv::Point3_<T1>(pt_T2[k].x, pt_T2[k].y, pt_T2[k].z);
            return pt_T1;
        };

        template <typename T>
        static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matd_eigen_cv(cv::Mat cvMat)
        {
            assert((cvMat.type() == CV_64FC1 && typeid(T) == typeid(double)) ||
                   (cvMat.type() == CV_32FC1 && typeid(T) == typeid(float)));
            // if ()
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> tmp;
            tmp.resize(cvMat.rows, cvMat.cols);
            for (int h = 0; h < cvMat.rows; h++)
                for (int w = 0; w < cvMat.cols; w++)
                    tmp(h, w) = cvMat.at<T>(h, w);
            return tmp;
        }
        template <typename T>
        static cv::Mat matd_cv_eigen(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> eigenMat)
        {
            cv::Mat_<T> tmp(eigenMat.rows(), eigenMat.cols());
            // tmp.create(eigenMat.rows(), eigenMat.cols(), T);
            for (size_t h = 0; h < eigenMat.rows(); h++)
                for (size_t w = 0; w < eigenMat.cols(); w++)
                    tmp(h, w) = eigenMat(h, w);
            return tmp;
        }

        // template <typename T>
        // static std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> cv2eigen_align(std::vector<cv::Point_<T>> p2T)
        // {
        //     std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> tmp;
        //     tmp.resize(p2T.size());
        //     for (size_t i = 0; i < tmp.size(); i++)
        //     {
        //         tmp[i](0) = p2T[i].x;
        //         tmp[i](1) = p2T[i].y;
        //     };
        //     return tmp;
        // }

        // template <typename T>
        // static std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> cv2eigen_align(std::vector<cv::Point3_<T>> p3T)
        // {
        //     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tmp;
        //     tmp.resize(p3T.size());
        //     for (size_t i = 0; i < tmp.size(); i++)
        //     {
        //         tmp[i](0) = p3T[i].x;
        //         tmp[i](1) = p3T[i].y;
        //         tmp[i](2) = p3T[i].z;
        //     };
        //     return tmp;
        // }
        // template <typename T>
        // static std::vector<Eigen::Vector3d> cv2eigen(std::vector<cv::Point3_<T>> p3T)
        // {
        //     std::vector<Eigen::Vector3d> tmp;
        //     tmp.resize(p3T.size());
        //     for (size_t i = 0; i < tmp.size(); i++)
        //     {
        //         tmp[i](0) = p3T[i].x;
        //         tmp[i](1) = p3T[i].y;
        //         tmp[i](2) = p3T[i].z;
        //     };
        //     return tmp;
        // }
        // template <typename T>
        // static cv::Mat eigen2cv(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> eigenMat)
        // {
        //     cv::Mat_<T> tmp(eigenMat.rows(), eigenMat.cols());
        //     // tmp.create(eigenMat.rows(), eigenMat.cols(), T);
        //     for (size_t h = 0; h < eigenMat.rows(); h++)
        //         for (size_t w = 0; w < eigenMat.cols(); w++)
        //             tmp(h, w) = eigenMat(h, w);
        //     return tmp;
        // }

        // static std::vector<Eigen::Vector2d> cv2eigen_64FC2(cv::Mat a)
        // {
        //     assert(a.channels() == 2);
        //     assert(a.type() == CV_64FC2);
        //     std::vector<Eigen::Vector2d> tmp;
        //     tmp.resize(a.rows);
        //     for (size_t h = 0; h < a.rows; h++)
        //         tmp[h] = Eigen::Vector2d(a.at<cv::Point2d>(h).x, a.at<cv::Point2d>(h).y);
        //     return tmp;
        // }
        // static std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> cv2eigen_64FC2_aligned(cv::Mat a)
        // {
        //     assert(a.channels() == 2);
        //     assert(a.type() == CV_64FC2);
        //     std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> tmp;
        //     tmp.resize(a.rows);
        //     for (size_t h = 0; h < a.rows; h++)
        //         tmp[h] = Eigen::Vector2d(a.at<cv::Point2d>(h).x, a.at<cv::Point2d>(h).y);
        //     return tmp;
        // }

        // static Sophus::SO3d cv2sophus(cv::Mat r)
        // {
        //     assert(r.type() == CV_64FC1);

        //     // cv::Mat R;
        //     // cv::Rodrigues(r, R);
        //     // std::cout << R << std::endl;
        //     assert(r.rows == 3);
        //     assert(r.cols == 3 || r.cols == 1);

        //     if (r.cols == 1 && r.rows == 3)
        //         return Sophus::SO3d::exp(cv2eigen<double>(r));
        //     else if (r.cols == 3 && r.rows == 3)
        //     {
        //         Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R = cv2eigen<double>(r);
        //         Eigen::Ref<Eigen::Matrix3d> R_fixed(R);
        //         return Sophus::SO3d::fitToSO3(R_fixed);
        //     }

        //     return Sophus::SO3d::rotX(0.0);
        // };
        // static Sophus::SE3d cv2sophus(cv::Mat r, cv::Mat t)
        // {
        //     assert(r.type() == CV_64FC1 && t.type() == CV_64FC1);

        //     return Sophus::SE3d(cv2sophus(r), cv2eigen<double>(t));
        // };
    };

} /* namespace abv_core */

#endif
