#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "calibrateCamera/Zhang.hpp"

/**
 * @brief https://learnopencv.com/understanding-lens-distortion/
 * @return int
 */
void display_resize(const cv::Mat &src, cv::Mat &dst, double scale = 0.75)
{
	cv::resize(src, dst, cv::Size(scale * src.cols, scale * src.rows));
}
void detect_and_calibrate(const std::vector<cv::Mat> &vec_mat_, const cv::Size &chessboard_size_)
{
	size_t n_views = 0;
	std::vector<Eigen::Matrix2Xd> views_image_p2ds;
	std::vector<Eigen::Matrix2Xd> views_board_p2ds;
	Eigen::Matrix2Xd bp = calibration_toolkit::createCheckerboardPoints(Eigen::Vector2i(chessboard_size_.width, chessboard_size_.height), 1.0);
	cv::Mat imageDisplay;
	for (const auto &img : vec_mat_)
	{
		std::vector<cv::Point2f> corner_pts;
		int found = cv::findChessboardCorners(img, chessboard_size_, corner_pts, cv::CALIB_CB_ACCURACY | cv::CALIB_CB_NORMALIZE_IMAGE);

		if (!found)
			continue;

		n_views++;
		// cv::TermCriteria criteria(cv::TermCriteria::EPS, 30, 0.001);
		// cv::cornerSubPix(img, corner_pts, chessboard_size_, cv::Size(11, 11), criteria);
		Eigen::Matrix2Xd image_p2ds;
		image_p2ds.resize(2, chessboard_size_.width * chessboard_size_.height);
		image_p2ds.setZero();
		for (int r = 0; r < corner_pts.size(); ++r)
		{
			image_p2ds(0, r) = (corner_pts)[r].x;
			image_p2ds(1, r) = (corner_pts)[r].y;
		}

		views_board_p2ds.push_back(bp);
		views_image_p2ds.push_back(image_p2ds);

		cv::drawChessboardCorners(img, chessboard_size_, corner_pts, found);
		display_resize(img, imageDisplay);

		cv::imshow("cvWindowName", imageDisplay);
		cv::waitKey(1);
	}
	printf("# [DEBUG] NUMBER OF VALID OBSERVATIONS %lu\n", n_views);
	Eigen::Matrix3d K;
	Eigen::VectorXd D;
	D.resize(5);
	D.setZero();
	double rmse = calibration_toolkit::calibrateCamera(views_image_p2ds, views_board_p2ds, K, D, calibration_toolkit::CalibFlags::CAMERA_K3P2_MULTIPLY | calibration_toolkit::CalibFlags::CAMERA_FIX_K3);
	printf("# [DEBUG] K AND D ==========================\n");
	std::cout << K << std::endl;
	std::cout << D << std::endl;
	printf("# [DEBUG] RMSE (PIXEL) =====================\n");
	std::cout << rmse << std::endl;
}
int main()
{
	// cv::CALIB_CB_ACCURACY
	FLAGS_log_dir = "./";
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging("calibrator");
	google::LogToStderr();
	std::vector<cv::String> imagesPaths;
	cv::glob("/media/czhou/BE_DATA/DataSets/EVO_BE/HawkEye/COMAC/EE0_mid/Cam_1/*.jpg", imagesPaths);
	std::vector<cv::Mat> images;
	// for (const auto &path : images)
	for (int g = 0; g < imagesPaths.size(); g = g + 1)
	{
		cv::Mat image = cv::imread(imagesPaths[g], cv::IMREAD_GRAYSCALE);
		if (!image.empty())
			images.push_back(image);
	}
	detect_and_calibrate(images, cv::Size(6, 4));

	// CameraCalibrator m;
	// Eigen::Matrix3d camera_matrix;
	// Eigen::VectorXd k;
	// std::vector<Eigen::MatrixXd> vec_extrinsics;

	// m.set_input(vec_mat, cv::Size{11, 8});
	// std::cout << "Done\n";
	// m.get_result(camera_matrix, k, vec_extrinsics);

	// std::cout << "camera_matrix:\n"
	// 		  << camera_matrix << std::endl;
	// std::cout << "k:\n"
	// 		  << k << std::endl;
	// for (int i=0;i<vec_extrinsics.size();++i)
	//{
	//	LOG(INFO) << "vec_extrinsics["<<i<<"]:\n" << vec_extrinsics[i] << std::endl;
	//}
	// Eigen::Matrix3d opencv_camera_matrix;
	// opencv_camera_matrix << 532.79536563, 0., 342.4582516,
	// 	0, 532.91928339, 233.90060514,
	// 	0, 0, 1;
	// std::cout << "opencv calibrateCamera api result:\n"
	// 		  << opencv_camera_matrix << std::endl;
	// std::cin.get();
}