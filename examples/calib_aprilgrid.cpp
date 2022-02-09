#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "calibrateCamera/Zhang.hpp"
#include "apriltags/gridDetector.hpp"
#include "common/mat_converter.hpp"
#include "common/common.hpp"
#include "cmdparser.hpp"

#define OPENCV_CALIB_DEBUG

void display_resize(const cv::Mat &src, cv::Mat &dst, double scale = 0.75);

void findAprilgridCorners(const cv::Mat &srcGray, std::vector<cv::Point2d> &imagePoints_p2d, std::vector<cv::Point3d> &objectPoints_p3d);

bool detect_ours(const cv::Mat &image, std::vector<Eigen::Matrix2Xd> &views_image_p2ds, std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
				 std::vector<std::vector<cv::Point2f>> &views_image_p2fs_cv, std::vector<std::vector<cv::Point3f>> &views_board_p3fs_cv);
double calibrate_ours(const std::vector<Eigen::Matrix2Xd> &views_image_p2ds, const std::vector<Eigen::Matrix2Xd> &views_board_p2ds, cv::Mat &K, cv::Mat &D);

void detect_and_calibrate(const std::vector<cv::Mat> &images, cv::Mat &Kout, cv::Mat &Dout);

void check_undistort(const std::vector<cv::Mat> &images, cv::Mat K, cv::Mat D, double scale = 0.5);

std::string path2Data = "/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE1/camera_wide/Cam_0";
std::string ext = "jpg";
size_t cameraID = 0;
calibration_toolkit::CalibFlags cameraModel = calibration_toolkit::CalibFlags::CAMERA_K3P2_MULTIPLY;
calibration_toolkit::MultiCamRig mcr;

void configure_parser(cli::Parser &parser)
{
	parser.set_optional<bool>("u", "undist", false, "Check undistortion");
	parser.set_required<std::string>("e", "extension", "Must be jpg, png or bmp");
	parser.set_required<std::string>("i", "input", "Path to input data, e.g. /media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE1/camera_wide");
	parser.set_required<size_t>("c", "camera", "Camera ID, must be 0, 1 or 2");
	parser.set_optional<int>("m", "model", 0, "0 = K3P2_MUL, 1 = K3P2_DIV, 2 = K6P2");
	parser.set_optional<std::string>("o", "output", "", "Output path to JSON");
	parser.set_optional<std::string>("n", "name", "", "Device name in JSON");
}
int main(int argc, char *argv[])
{
	cli::Parser parser(argc, argv);
	configure_parser(parser);
	parser.run_and_exit_if_error();
	ext = parser.get<std::string>("e");
	path2Data = parser.get<std::string>("i");
	cameraID = parser.get<size_t>("c");

	printf("# [INFO] Image format = %s\n", ext.c_str());
	printf("# [INFO] Input path   = %s\n", path2Data.c_str());
	printf("# [INFO] Camera id    = %ld\n", cameraID);
	if (parser.get<int>("m") >= 0 && parser.get<int>("m") <= 2)

		switch (parser.get<int>("m"))
		{
		case 0:
			cameraModel = calibration_toolkit::CalibFlags::CAMERA_K3P2_MULTIPLY;
			printf("# [INFO] Distortion model = K3P2_MULTIPLY\n");
			break;
		case 1:
			cameraModel = calibration_toolkit::CalibFlags::CAMERA_K3P2_DIVIDE;
			printf("# [INFO] Distortion model = K3P2_DIVIDE\n");

			break;
		case 2:
			cameraModel = calibration_toolkit::CalibFlags::CAMERA_K6P2;
			printf("# [INFO] Distortion model = K6P2\n");
			break;
		default:
			break;
		}
	if (parser.doesArgumentExist("u", "undist"))
		printf("# [INFO] Check undistion  = True\n");
	if (!parser.get<std::string>("o").empty())
	{
		printf("# [INFO] Output result to = %s\n", parser.get<std::string>("o").c_str());
		if (parser.get<std::string>("n").empty())
		{
			printf("# [WARNING] Must assign device name in output JSON\n");
			return EXIT_FAILURE;
		}
		else

			printf("# [INFO] Device name      = %s\n", parser.get<std::string>("n").c_str());
	}

	// if (argc != 3)
	// {
	// 	printf("Usage\n  aprilgrid [path] [extension]\n");
	// 	return 0;
	// }
	// else
	// {
	// 	path2Data = argv[1];
	// 	ext = argv[2];
	// }
	FLAGS_log_dir = "./";
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging("calibrator");
	google::LogToStderr();
	std::vector<cv::String> imagesPaths;

	cv::glob(path2Data + "/Cam_" + std::to_string(cameraID) + "/*." + ext, imagesPaths);
	// cv::glob("/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE0/camera_mid/Cam_1/*.jpg", imagesPaths);
	std::vector<cv::Mat> images;
	for (int g = 0; g < imagesPaths.size(); g = g + 1)
	{
		cv::Mat image = cv::imread(imagesPaths[g], cv::IMREAD_GRAYSCALE);
		images.push_back(image);
	}
	// cv::Mat K, D;
	if (!parser.get<std::string>("o").empty())
	{
		mcr.readJSON(parser.get<std::string>("o"), parser.get<std::string>("n"));
	}
	detect_and_calibrate(images, mcr.K[cameraID], mcr.D[cameraID]);
	if (parser.doesArgumentExist("u", "undist"))
		// printf("# Output result to = %s\n", parser.get<std::string>("o").c_str());
		check_undistort(images, mcr.K[cameraID], mcr.D[cameraID]);

	if (!parser.get<std::string>("o").empty())
	{

		mcr.writeJSON(parser.get<std::string>("o"), parser.get<std::string>("n"));
	}
	return 0;
}

void display_resize(const cv::Mat &src, cv::Mat &dst, double scale)
{
	cv::resize(src, dst, cv::Size(scale * src.cols, scale * src.rows));
}

void findAprilgridCorners(const cv::Mat &srcGray, std::vector<cv::Point2d> &imagePoints_p2d, std::vector<cv::Point3d> &objectPoints_p3d)
{
	double length = 0.088;
	double ratio = 0.3;
	calibration_toolkit::AprilgridDetector::AprilgridOptions opt;
	opt.subpixelWindowSize = 5.0;
	calibration_toolkit::AprilgridDetector detector(length, ratio, opt);
	Eigen::MatrixXd imagePoints_m2d;
	std::vector<bool> imagePoints_flags;
	detector.computeObservation(srcGray, imagePoints_m2d, imagePoints_flags);
	detector.convertResults_OpenCV(imagePoints_m2d, imagePoints_flags, objectPoints_p3d, imagePoints_p2d);
}

bool detect_ours(const cv::Mat &image, std::vector<Eigen::Matrix2Xd> &views_image_p2ds, std::vector<Eigen::Matrix2Xd> &views_board_p2ds,
				 std::vector<std::vector<cv::Point2f>> &views_image_p2fs_cv, std::vector<std::vector<cv::Point3f>> &views_board_p3fs_cv)
{
	std::vector<cv::Point2d> imagePoints_p2d;
	std::vector<cv::Point3d> objectPoints_p3d;
	findAprilgridCorners(image, imagePoints_p2d, objectPoints_p3d);
	if (objectPoints_p3d.size() < 4)
		return false;
	Eigen::Matrix2Xd image_p2ds, board_p2ds;
	image_p2ds.resize(2, imagePoints_p2d.size());
	image_p2ds.setZero();
	board_p2ds.resize(2, objectPoints_p3d.size());
	board_p2ds.setZero();
	for (int r = 0; r < imagePoints_p2d.size(); ++r)
	{
		image_p2ds(0, r) = (imagePoints_p2d)[r].x;
		image_p2ds(1, r) = (imagePoints_p2d)[r].y;
		board_p2ds(0, r) = (objectPoints_p3d)[r].x;
		board_p2ds(1, r) = (objectPoints_p3d)[r].y;
	}
	views_board_p2ds.push_back(board_p2ds);
	views_image_p2ds.push_back(image_p2ds);
	views_image_p2fs_cv.push_back(calibration_toolkit::MatConverter::cvPoint2<float, double>(imagePoints_p2d));
	views_board_p3fs_cv.push_back(calibration_toolkit::MatConverter::cvPoint3<float, double>(objectPoints_p3d));
	return true;
}
double calibrate_ours(const std::vector<Eigen::Matrix2Xd> &views_image_p2ds, const std::vector<Eigen::Matrix2Xd> &views_board_p2ds, cv::Mat &K, cv::Mat &D)
{
	Eigen::Matrix3d K_eigen;
	Eigen::VectorXd D_eigen;
	double rmse = calibration_toolkit::calibrateCamera(views_image_p2ds, views_board_p2ds, K_eigen, D_eigen, cameraModel);

	K = calibration_toolkit::MatConverter::matd_cv_eigen<double>(K_eigen).clone();
	D = calibration_toolkit::MatConverter::matd_cv_eigen<double>(D_eigen).clone();
	return rmse;
}
/**
 * @brief https://learnopencv.com/understanding-lens-distortion/
 * @return int
 */
void detect_and_calibrate(const std::vector<cv::Mat> &images, cv::Mat &Kout, cv::Mat &Dout)
{
	// size_t n_views = 0;

	std::vector<Eigen::Matrix2Xd> views_image_p2ds;
	std::vector<Eigen::Matrix2Xd> views_board_p2ds;
	std::vector<std::vector<cv::Point2f>> views_image_p2fs_cv;
	std::vector<std::vector<cv::Point3f>> views_board_p3fs_cv;
	cv::Mat K_ours, D_ours;

	// DETECTION
	for (const auto &img : images)
		detect_ours(img, views_image_p2ds, views_board_p2ds, views_image_p2fs_cv, views_board_p3fs_cv);

	// OUR CALIBRATE CAMERA
	double rmse_ours = calibrate_ours(views_image_p2ds, views_board_p2ds, K_ours, D_ours);

	std::cout << K_ours << "\n";
	std::cout << D_ours << "\n";
	std::cout << rmse_ours << "\n";

#ifndef OPENCV_CALIB_DEBUG
	cv::Size imageSize = images[0].size();
	cv::Mat K_cv, D_cv;
	std::vector<cv::Mat> rvecs, tvecs;

	double rmse_cv = cv::calibrateCamera(views_board_p3fs_cv, views_image_p2fs_cv, imageSize, K_cv, D_cv, rvecs, tvecs, cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_RATIONAL_MODEL);
	rmse_cv = cv::calibrateCamera(views_board_p3fs_cv, views_image_p2fs_cv, imageSize, K_cv, D_cv, rvecs, tvecs, cv::CALIB_FIX_INTRINSIC | cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL);
	std::cout << K_cv << "\n";
	std::cout << D_cv << "\n";
	std::cout << rmse_cv << "\n";
#endif
	K_ours.copyTo(Kout);
	D_ours.copyTo(Dout);

	// OPENCV UNDISTORTION

	return;
}

void check_undistort(const std::vector<cv::Mat> &images, cv::Mat K, cv::Mat D, double scale)
{
	cv::Mat imageDisplay;
	// cv::Mat K_out = K_ours, D_out = D_ours;
	for (const auto &img : images)
	{
		cv::Mat undist;
		cv::Mat K_new = K.clone();
		cv::Mat D_new = D.clone();
		// K_new = K_new * 0.5;
		K_new.at<double>(0, 0) *= scale;
		K_new.at<double>(1, 1) *= scale;
		cv::undistort(img, undist, K, D, K_new);
		display_resize(undist, imageDisplay);
		cv::imshow("CHECK UNDISTORTION", imageDisplay);
		cv::waitKey(0);
	}
}