#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <vector>
//#include "calibrateCamera/Zhang.hpp"
#include "apriltags/gridDetector.hpp"
#include "common/mat_converter.hpp"
//#include "handeye/handeye.hpp"
#include "common/io.hpp"
#include "common/load_data_be_csv.hpp"
#include "common/common.hpp"
#include "kinematics/kinematics.hpp"

double X_M1_data_T[9] = {0.9999705316462844, -0.005533307937304417, -0.005321498127264511,
						 0.005404070518134445, 0.9996973254963561, -0.02400111283393941,
						 0.005452692993608376, 0.02397164780951499, 0.9996977684482515};

double X_M1_data_H[9] = {0.9999705142801161, -0.0055342230228593, -0.005323809340441836,
						 0.005404913924511155, 0.9996972504299189, -0.02400404941522757,
						 0.005455041322368429, 0.02397456690731484, 0.9996976856359016};
cv::Mat X_M1(3, 3, CV_64FC1, X_M1_data_T);

void display_resize(const cv::Mat &src, cv::Mat &dst, double scale = 0.75)
{
	cv::resize(src, dst, cv::Size(scale * src.cols, scale * src.rows));
}

void detect_and_calibrate(cv::String beSaveDataPath,
						  const calibration_toolkit::MultiCamRig &mcr,
						  const calibration_toolkit::KinematicsX1X2 &km)
{
	calibration_toolkit::RecordData_BECSV data(beSaveDataPath, calibration_toolkit::RecordDataHeader_BECSV::jpg);
	cv::Mat K = mcr.K[0];
	cv::Mat D = mcr.D[0];
	// std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;
	cv::Mat image_ud, mask;
	cv::Mat image_stitch, warp_mask_b_e, warp_b_e, image_sum, image_stitch_wireframe;
	int pad = 1000;
	int good_count = 0;
	// warp_mask_b_e.create()
	for (long i = data.idMin(); i <= data.idMax(); i++)
	{
		calibration_toolkit::Frame_BE frame;
		if (data.grabFrame(i, 0, frame, calibration_toolkit::RecordDataHeader_BECSV::ENCODER_DEG, true))
		{
			cv::Size stitch_size(frame.image.cols + pad * 2, frame.image.rows + pad * 2);
			if (mask.empty())
				mask.create(frame.image.size(), CV_8UC1);
			if (image_stitch.empty())
			{
				image_stitch.create(stitch_size, CV_8UC3);
				image_stitch_wireframe.create(stitch_size, CV_8UC3);
				image_stitch_wireframe.setTo(cv::Scalar(255, 255, 255));
			}
			if (image_sum.empty())
				image_sum.create(stitch_size, CV_32FC3);

			mask.setTo(cv::Scalar(255));
			// std::cout << "# NEW FRAME " << frame.id << " ##############################################################################" << std::endl;
			double theta1 = frame.motorEncoder(5) / 180.0 * CV_PI; // yaw
			double theta2 = frame.motorEncoder(3) / 180.0 * CV_PI; // pitch
			Eigen::VectorXd ts;
			ts << theta1, theta2;

			if (ts.norm() > 30.0 / 180.0 * CV_PI)
				continue;
			// km.reset();
			good_count++;
			Eigen::Matrix3d R_b_e = km.FK(ts).block<3, 3>(0, 0);
			// R_b_e = R_b_e;
			// std::cout << ts.transpose() / CV_PI * 180.0 << std::endl;
			// std::cout << R_b_e << std::endl;

			cv::Mat H_b_e = X_M1.t() * calibration_toolkit::MatConverter::matd_cv_eigen<double>(R_b_e) * X_M1;
			cv::Mat K_shift = K.clone();
			K_shift.at<double>(0, 2) += pad * 1.2;
			K_shift.at<double>(1, 2) += pad;
			H_b_e = K_shift * H_b_e * K.inv();
			cv::Mat image = frame.image;
			cv::Mat image_display;
			// cv::Mat image_ud, warp_b_e;

			cv::undistort(image, image_ud, K, D);

			cv::Mat lookUpTable(1, 256, CV_8U);
			uchar *p = lookUpTable.ptr();
			for (int i = 0; i < 256; ++i)
				p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 0.75) * 255.0);
			cv::Mat res = image_ud.clone();
			LUT(image_ud, lookUpTable, res);
			res.copyTo(image_ud);
			warp_b_e = image_stitch.clone();
			cv::warpPerspective(image_ud, warp_b_e, H_b_e, warp_b_e.size(), cv::INTER_NEAREST);
			cv::warpPerspective(mask, warp_mask_b_e, H_b_e, warp_b_e.size());
			std::vector<cv::Point2d> pt, pt_warp;
			// std::vector<double> d;sti
			pt.push_back(cv::Point2d(0, 0));
			pt.push_back(cv::Point2d(image_ud.cols, 0));
			pt.push_back(cv::Point2d(image_ud.cols, image_ud.rows));
			pt.push_back(cv::Point2d(0, image_ud.rows));
			// cv::ImreadModes::gray
			//  pt.pushback()

			cv::perspectiveTransform(pt, pt_warp, H_b_e);

			warp_b_e.copyTo(image_stitch, warp_mask_b_e);
			cv::Mat warp_b_e_32fc;
			warp_b_e.convertTo(warp_b_e_32fc, CV_32FC3);
			image_sum += warp_b_e_32fc;
			cv::Scalar clr(std::rand() % 255, std::rand() % 255, std::rand() % 255);
			int tk = 5;
			if (ts.norm() < 3.9 / 180.0 * CV_PI)
			{
				tk = tk * 8;
			}

			for (size_t j = 0; j < 4; j++)
				cv::line(image_stitch_wireframe, pt_warp[j], pt_warp[(j + 1) % 4], clr, tk);

			display_resize(image_stitch, image_display, 0.5);
			// cv::imshow("STITCHING", image_display);
			cv::waitKey(1);
		}
	}
	image_sum *= (1.0 / good_count);
	cv::Mat image_sum_display;
	cv::Mat image_sum_out;
	cv::normalize(image_sum, image_sum, 0, 1, cv::NORM_MINMAX);
	image_sum.convertTo(image_sum_out, CV_8UC3, 255.0);
	display_resize(image_sum, image_sum_display, 0.5);
	cv::imshow("STITCHING", image_sum_display);
	cv::imwrite("/media/czhou/DATA/EE/Indoor_00/Output/stitch_M1_H.png", image_sum_out);

	cv::waitKey(0);

	return;
}

int main()
{

	calibration_toolkit::MultiCamRig mcr;

	mcr.readJSON("./ee1.json", "BE310A28210001");
	// mcr.read(mcr.json, "BE1", "K", &(mcr.K[0]), 3, 3);
	// std::cout << mcr.K[0] << std::endl;
	// std::cout << mcr.D[0] << std::endl;
	// std::cout << mcr.X[0] << std::endl;
	// std::cout << mcr.X[1] << std::endl;
	FLAGS_log_dir = "./";
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging("calibrator");
	google::LogToStderr();
	Eigen::Matrix4d X[2];
	X[0] = calibration_toolkit::MatConverter::matd_eigen_cv<double>(mcr.X[0]);
	X[1] = calibration_toolkit::MatConverter::matd_eigen_cv<double>(mcr.X[1]);

	calibration_toolkit::KinematicsX1X2 km(&X[0]);
	km.reset();
	detect_and_calibrate(std::string("/media/czhou/DATA/EE/Indoor_00/1_calib/camera_tele"), mcr, km);
}