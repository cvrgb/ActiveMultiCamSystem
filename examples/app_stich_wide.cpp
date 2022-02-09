#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "calibrateCamera/Zhang.hpp"
#include "apriltags/gridDetector.hpp"
#include "common/mat_converter.hpp"

#include "handeye/handeye.hpp"
#include "common/io.hpp"
#include "common/load_data_be_csv.hpp"
#include "common/common.hpp"
#include "kinematics/kinematics.hpp"

void display_resize(const cv::Mat &src, cv::Mat &dst, double scale = 0.75)
{
	cv::resize(src, dst, cv::Size(scale * src.cols, scale * src.rows));
}

/**
 * @brief https://learnopencv.com/understanding-lens-distortion/
 * @return int
 */
void detect_and_calibrate(cv::String beSaveDataPath,
						  const calibration_toolkit::MultiCamRig &mcr,
						  const calibration_toolkit::KinematicsX1X2 &km)
{
	// size_t n_views = 0;
	// std::vector<Eigen::Matrix2Xd> views_image_p2ds;
	// std::vector<Eigen::Matrix2Xd> views_board_p2ds;
	// std::vector<std::vector<cv::Point2f>> views_image_p2fs_cv;
	// std::vector<std::vector<cv::Point3f>> views_board_p3fs_cv;
	// std::vector<std::vector<cv::Point2f>> views_image_norm_p2fs_cv;
	// std::vector<Eigen::Matrix2Xd> views_image_norm_p2ds;
	// std::vector<Eigen::Matrix3Xd> views_board_p3ds;
	// std::vector<Eigen::Vector2d> thetas;
	// const  &thetas,
	// std::string calibrationDir = "/usr/Evo_BionicEyes/io_path/fastSave";
	// std::string beSaveDataPath = "/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE1/motor_wide";
	calibration_toolkit::RecordData_BECSV data(beSaveDataPath, calibration_toolkit::RecordDataHeader_BECSV::jpg);

	// Eigen::Matrix4d X1, X2, L;
	// // X1.setIdentity();
	// // X2.setIdentity();
	// // X1.setIdentity();
	// // X1.block<3, 3>(0, 0) = Eigen::AngleAxisd(1.92479, Eigen::Vector3d(-0.684465, -0.506321, 0.524544)).toRotationMatrix(); // Yaw
	// // X2.block<3, 3>(0, 0) = Eigen::AngleAxisd(1.73774, Eigen::Vector3d(0.363011, -0.854423, 0.371732)).toRotationMatrix();  // Pitch
	// X1 << -0.237271437689159, -0.0208968322083928, -0.971218609408084,
	// 	-32.469213215203, 0.971402740863793, -0.0142499514089653,
	// 	-0.237009818208345, 50.653999556616, -0.00888706358870689,
	// 	-0.999680079469675, 0.0236803465404098, 0.598699012346748, 0, 0, 0, 1;
	// X1.block<3, 1>(0, 3) = 5 * Eigen::Vector3d::Random();

	// X2 << -0.0129208122517066, -0.749981951134545, -0.661332084193093,
	// 	-35.7949207979476, 0.00437531610054809, 0.661338557715894,
	// 	-0.750074775397282, 7.79212334387461, 0.999906950280765,
	// 	-0.0125851122634271, -0.00526362323314267, -0.163265468864144, 0, 0, 0, 1;
	// X2.block<3, 1>(0, 3) = 5 * Eigen::Vector3d::Random();
	// L << -0.998492687739403, -0.0528576543290368, 0.0147790699901552, 375.343685932303,
	// 	0.0516504683358182, -0.996015565505672, -0.0726995350130025, -119.007330513521,
	// 	0.0185629106454913, -0.0718266082259785, 0.997244381633268, 2809.02038560834,
	// 	0, 0, 0, 1;

	cv::Mat K = mcr.K[0];
	cv::Mat D = mcr.D[0];

	std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;
	cv::Mat image_ud, mask;
	cv::Mat image_stitch, warp_mask_b_e, warp_b_e;
	int pad = 1500;
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
				image_stitch.create(stitch_size, CV_8UC3);

			mask.setTo(cv::Scalar(255));
			// std::cout << "# NEW FRAME " << frame.id << " ##############################################################################" << std::endl;
			double theta1 = frame.motorEncoder(5) / 180.0 * CV_PI; // yaw
			double theta2 = frame.motorEncoder(3) / 180.0 * CV_PI; // pitch
			Eigen::VectorXd ts;

			ts.resize(2);
			ts(0) = theta1;
			ts(1) = theta2;
			Eigen::Matrix3d R_b_e = km.FK(ts).block<3, 3>(0, 0);
			std::cout << ts.transpose() / CV_PI * 180.0 << std::endl;
			// std::cout << R_b_e << std::endl;

			cv::Mat H_b_e = calibration_toolkit::MatConverter::matd_cv_eigen<double>(R_b_e);
			cv::Mat K_shift = K.clone();
			K_shift.at<double>(0, 2) += pad * 1.2;
			K_shift.at<double>(1, 2) += pad;
			H_b_e = K_shift * H_b_e * K.inv();
			cv::Mat image = frame.image;
			cv::Mat image_display;
			// cv::Mat image_ud, warp_b_e;

			cv::undistort(image, image_ud, K, D);
			warp_b_e = image_stitch.clone();
			cv::warpPerspective(image_ud, warp_b_e, H_b_e, warp_b_e.size(), cv::INTER_NEAREST);
			cv::warpPerspective(mask, warp_mask_b_e, H_b_e, warp_b_e.size());
			std::vector<cv::Point2d> pt, pt_warp;
			// std::vector<double> d;sti
			pt.push_back(cv::Point2d(0, 0));
			pt.push_back(cv::Point2d(image_ud.cols, 0));
			pt.push_back(cv::Point2d(image_ud.cols, image_ud.rows));
			pt.push_back(cv::Point2d(0, image_ud.rows));

			// pt.pushback()
			cv::perspectiveTransform(pt, pt_warp, H_b_e);

			warp_b_e.copyTo(image_stitch, warp_b_e);
			cv::Scalar clr(std::rand() % 255, std::rand() % 255, std::rand() % 255);
			for (size_t j = 0; j < 4; j++)
				cv::line(image_stitch, pt_warp[j], pt_warp[(j + 1) % 4], clr, 2);

			display_resize(image_stitch, image_display, 0.3);

			cv::imshow("STITCHING", image_display);
			cv::waitKey(10);

			// if (detect_ours(imageGray, views_image_p2ds, views_board_p2ds, views_image_p2fs_cv, views_board_p3fs_cv))
			// {
			// 	std::vector<cv::Point2f> tmp;
			// 	cv::undistortPoints(views_image_p2fs_cv.back(), tmp, K_input, D_input);
			// 	thetas.push_back(Eigen::Vector2d(theta1, theta2));
			// 	views_image_norm_p2fs_cv.push_back(tmp);
			// }
		}
	}
	cv::waitKey(0);
	// std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;

	// size_t n_views = views_board_p2ds.size();
	// views_board_p3ds.resize(n_views);
	// views_image_norm_p2ds.resize(n_views);
	// for (size_t k = 0; k < n_views; k++)
	// {
	// 	// std::cout << views_board_p2ds[k].cols() << std::endl;
	// 	size_t n_detected = views_board_p2ds[k].cols();
	// 	views_board_p3ds[k].resize(3, n_detected);
	// 	views_board_p3ds[k].setZero();
	// 	views_board_p3ds[k].row(0) = views_board_p2ds[k].row(0);
	// 	views_board_p3ds[k].row(1) = views_board_p2ds[k].row(1);
	// 	views_image_norm_p2ds[k].resize(2, n_detected);

	// 	for (size_t b = 0; b < n_detected; b++)
	// 	{
	// 		views_image_norm_p2ds[k](0, b) = views_image_norm_p2fs_cv[k][b].x;
	// 		views_image_norm_p2ds[k](1, b) = views_image_norm_p2fs_cv[k][b].y;
	// 	}
	// 	// views_image_norm_p2fs[k].row(0) = views_board_p2ds[k].row(0);
	// 	// views_image_norm_p2fs[k].row(1) = views_board_p2ds[k].row(1);
	// }

	// std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;

	// calibration_toolkit::kba_X1X2L(views_image_norm_p2ds, views_board_p3ds, thetas, X1, X2, L);

	return;
}

int main()
{

	calibration_toolkit::MultiCamRig mcr;

	mcr.readJSON("./ee1.json", "BE310A28210001");
	// mcr.read(mcr.json, "BE1", "K", &(mcr.K[0]), 3, 3);
	std::cout << mcr.K[0] << std::endl;
	std::cout << mcr.D[0] << std::endl;
	std::cout << mcr.X[0] << std::endl;
	std::cout << mcr.X[1] << std::endl;

	FLAGS_log_dir = "./";
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging("calibrator");
	google::LogToStderr();
	Eigen::Matrix4d X[2];
	X[0] = calibration_toolkit::MatConverter::matd_eigen_cv<double>(mcr.X[0]);
	X[1] = calibration_toolkit::MatConverter::matd_eigen_cv<double>(mcr.X[1]);

	calibration_toolkit::KinematicsX1X2 km(&X[0]);

	detect_and_calibrate(std::string("/media/czhou/DATA/EE/Indoor_00/1_calib/camera_tele"), mcr, km);
}