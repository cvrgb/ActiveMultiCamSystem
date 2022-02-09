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

#ifndef ABV_CORE_UTILS_HELPER_HPP
#define ABV_CORE_UTILS_HELPER_HPP

#include <opencv2/opencv.hpp>
// #include <Eigen/Core>
// #include <typeinfo>
// #include <sophus/so3.hpp>
// #include <sophus/se3.hpp>

namespace abv_core
{

    class Helper
    {
    public:
        static void anaglyphFromStereo(cv::Mat &srcImage)
        {
            int imgRows = srcImage.rows;
            int imgCols = srcImage.cols / 2;
            int n_ch = srcImage.channels();

            if (n_ch == 3)
            {
                std::vector<cv::Mat> ch(3);
                cv::split(srcImage, ch);

                ch[0].colRange(imgCols * 0, imgCols * 1).copyTo(ch[0].colRange(imgCols * 1, imgCols * 2));
                ch[1].colRange(imgCols * 0, imgCols * 1).copyTo(ch[1].colRange(imgCols * 1, imgCols * 2));
                merge(ch, srcImage);
            }
            else if (n_ch == 1)
                for (int i = 0; i < imgRows / 2; i++)
                    srcImage.row(2 * i).colRange(0, imgCols).copyTo(srcImage.row(2 * i).colRange(imgCols, imgCols * 2));
        };
        static void checkStereoRectify(const cv::Mat &imgLeft, const cv::Mat &imgRight, float scale)
        {
            const std::string CV_WINDOW_NAME("[DEBUG] Check stereorectify");
            const int lineGap = 25;

            cv::Mat stereo(imgRight.rows, imgRight.cols * 2, imgRight.type());
            cv::Mat stereo_show;
            cv::namedWindow(CV_WINDOW_NAME, 1);

            //if (imgLeft.depth() < 1)
            //{
            //imgLeft.convertTo(imgLeft, CV_64FC3);
            //}

            imgRight.copyTo(stereo.colRange(imgRight.cols, imgRight.cols * 2));
            imgLeft.copyTo(stereo.colRange(0, imgRight.cols));
            cv::resize(stereo, stereo_show, cv::Size(0, 0), scale, scale);
            for (int i = 0; i <= stereo_show.rows; i += lineGap)
                cv::line(stereo_show, cv::Point(0, i), cv::Point(stereo_show.cols, i), cv::Scalar(0, 0, 255));
            cv::imshow(CV_WINDOW_NAME, stereo_show);
            //cv::moveWindow("w", 800, 500);

            //stereo.release();
            //stereo_show.release();
        }
        static void rectifyUndistortRemap(cv::Mat R10, cv::Mat t10,
                                          cv::Mat K0, cv::Mat D0, cv::Mat K1, cv::Mat D1, double scale,
                                          const cv::Mat &srcImage0, const cv::Mat &srcImage1,
                                          cv::Mat dstImage0, cv::Mat dstImage1)
        {

            //cv::Matx33d K1 = this->K(this->eyeIndex().at("Left"), scale);
            //auto D1 = this->D(this->eyeIndex().at("Left"));
            //auto K2 = this->K(this->eyeIndex().at("Right"), scale);
            //auto D2 = this->D(this->eyeIndex().at("Right"));
            const auto imageSize = srcImage0.size();
            cv::Matx33d R0, R1;
            cv::Matx34d P0, P1;
            cv::Matx44d Q;
            cv::stereoRectify(K0, D0, K1, D1, imageSize, R10, t10, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 1);
            cv::Mat rmap[2][2];
            //cv::initUndistortRectifyMap(K1,D1,R1,P1)
            cv::initUndistortRectifyMap(K0, D0, R0, P0, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
            cv::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
            //RLeft = R1;
            //PLeft = P1;
            //QOut = Q;
            //std::cout << P1 << "\n"
            cv::remap(srcImage0, dstImage0, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
            cv::remap(srcImage1, dstImage1, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
            // std::cout << "[DEBUG] -------P1---------------------\n";
            // std::cout << P1 << "\n";
            // std::cout << "[DEBUG] -------P2---------------------\n";
            // std::cout << P2 << "\n";
            // std::cout << "[DEBUG] -------Q---------------------\n";
            // std::cout << Q << "\n";
            // std::cout << Q(2, 3) << "\n";
            // std::cout << Q(3, 2) << "\n";
            // std::cout << Q(2, 3) / Q(3, 2) << "\n";
            // std::cout << Q(3, 2) << "\n";
            // std::cout << "[DEBUG] -------RIGHT---------------------\n";
        };
        static double distSampson(
            std::vector<std::vector<cv::Point2d>> imagePoint0,
            std::vector<std::vector<cv::Point2d>> imagePoint1,
            cv::Mat K0, cv::Mat D0, cv::Mat K1, cv::Mat D1,
            cv::Mat R, cv::Mat t, int &nPoints)
        {
            cv::Mat tnorm;
            cv::normalize(t, tnorm);
            cv::Mat T = cv::Mat::zeros(3, 3, CV_64FC1);
            T.at<double>(1, 2) = -tnorm.at<double>(0, 0);
            T.at<double>(0, 2) = tnorm.at<double>(1, 0);
            T.at<double>(0, 1) = -tnorm.at<double>(2, 0);
            T.at<double>(2, 1) = -T.at<double>(1, 2);
            T.at<double>(2, 0) = -T.at<double>(0, 2);
            T.at<double>(1, 0) = -T.at<double>(0, 1);
            cv::Mat E = T * R;
            cv::Mat F = K1.t().inv() * E * K0.inv();
            cv::Mat i3 = cv::Mat::eye(3, 3, CV_64FC1);
            double error = 0;
            int count = 0;
            for (size_t i = 0; i < imagePoint0.size(); i++)
                for (size_t j = 0; j < imagePoint0[i].size(); j++)
                {
                    //std::cin.get();
                    cv::Point2d ip0 = imagePoint0[i][j];
                    cv::Point2d ip0_ud;
                    cv::Mat p0(ip0);
                    cv::Mat p0_ud;
                    cv::undistortPoints(p0, p0_ud, K0, D0, i3, K0);
                    cv::Point3d h0_ud(p0_ud.at<double>(0, 0), p0_ud.at<double>(0, 1), 1.0);

                    cv::Point2d ip1 = imagePoint1[i][j];
                    cv::Point2d ip1_ud;
                    cv::Mat p1(ip1);
                    cv::Mat p1_ud;
                    cv::undistortPoints(p1, p1_ud, K1, D1, i3, K1);
                    cv::Point3d h1_ud(p1_ud.at<double>(0, 0), p1_ud.at<double>(0, 1), 1.0);

                    cv::Mat m0(h0_ud);
                    cv::Mat m1(h1_ud);

                    error += cv::sampsonDistance(m0, m1, F);
                    count++;
                };
            nPoints = count;
            return sqrt(error / count);
        }
        template <typename T>
        static double dist_cvPoint2Border(std::vector<cv::Point_<T>> pts, size_t rows, size_t cols)
        {

            double dist_min[4] = {1e8, 1e8, 1e8, 1e8};
            for (size_t k = 0; k < pts.size(); k++)
            {
                cv::Point2d a;
                double dist[4] = {0, 0, 0, 0};
                dist[0] = std::max(pts[k].y, (T)0.0);           //tiop
                dist[1] = std::max((T)rows - pts[k].y, (T)0.0); //bottom
                dist[2] = std::max(pts[k].x, (T)0.0);           //left
                dist[3] = std::max((T)cols - pts[k].x, (T)0.0); //right
                for (int j = 0; j < 4; j++)
                    if (dist[j] < dist_min[j])
                        dist_min[j] = dist[j];
            }
            double min = dist_min[0];
            for (int j = 1; j < 4; j++)
            {
                if (dist_min[j] < min)
                    min = dist_min[j];
            }
            return min;
        };
    };

} /* namespace abv_core */

#endif
