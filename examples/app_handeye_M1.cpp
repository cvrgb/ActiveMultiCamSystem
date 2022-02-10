#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "calibrateCamera/Zhang.hpp"
#include "apriltags/gridDetector.hpp"
#include "common/mat_converter.hpp"

#include "handeye/handeye.hpp"
#include "common/io.hpp"
#include "common/load_data_be_csv.hpp"
#include "common/common.hpp"
#include "kinematics/kinematics.hpp"

bool estimateRtFromPlanar(std::vector<cv::Point2d> corners,
						  std::vector<cv::Point3d> tmp,
						  cv::Mat K, cv::Mat D,
						  cv::Mat &Rout, cv::Mat &tOut)
{
	// std::vector<cv::Point3d> tmp;
	// create3DCorners(patternSize, squareSize, tmp);
	cv::Mat zeros31 = cv::Mat::zeros(3, 1, CV_64FC1);

	std::vector<cv::Mat> rvecIPPE, tvecIPPE;
	rvecIPPE.resize(2);
	tvecIPPE.resize(2);
	rvecIPPE[0] = zeros31.clone();
	rvecIPPE[1] = zeros31.clone();
	tvecIPPE[0] = zeros31.clone();
	tvecIPPE[1] = zeros31.clone();
	tvecIPPE[0].at<double>(3, 0) = 2500.0f;
	tvecIPPE[1].at<double>(3, 0) = 2500.0f;

	cv::Mat rvec, tvec, inl0, inl1, inl;
	cv::solvePnPGeneric(tmp, corners, K, D, rvecIPPE, tvecIPPE, false, cv::SolvePnPMethod::SOLVEPNP_SQPNP);

	// cv::solvePnPGeneric
	rvec = rvecIPPE[0];
	tvec = tvecIPPE[0];
	Rout = rvec.clone();
	tOut = tvec.clone();
	std::vector<cv::Point2d> corners_reproj;
	cv::projectPoints(tmp, rvec, tvec, K, D, corners_reproj);
	int count = 0;
	for (size_t k = 0; k < corners.size(); k++)
	{
		cv::Point2d error = corners[k] - corners_reproj[k];
		if (cv::norm(error) <= 2.0)
			count++;
	}

	cv::solvePnPRansac(tmp, corners, K, D, rvec, tvec, true, 400, 2.0f, 0.99, inl0, cv::SolvePnPMethod::SOLVEPNP_AP3P);
	std::cout << "# INLINER NUMBER = " << inl0.rows << std::endl;
	std::cout << count << std::endl;
	std::cout << corners.size() << std::endl;
	if ((double)count / corners.size() < 0.75)
		return false;
	else
		return true;
	if (inl0.rows < 0.5 * corners.size())
	{
		rvec = rvecIPPE[1];
		tvec = tvecIPPE[1];

		// cv::solvePnPRansac(tmp, corners, K, D, rvec, tvec, true, 400, 2.0f, 0.99, inl1, cv::SolvePnPMethod::SOLVEPNP_AP3P);
		//  std::cout << "# INLINER NUMBER = " << inl1.rows << std::endl;
		inl = inl1;
	}
	else
		inl = inl0;

	// std::vector<cv::Point2d> corners_inl;
	// std::vector<cv::Point3d> objw_inl;
	// for (int g = 0; g < inl.rows; g++)
	// {
	//   corners_inl.push_back(corners[inl.at<int>(g, 0)]);
	//   objw_inl.push_back(tmp[inl.at<int>(g, 0)]);
	// }
	// std::cout << "INLN" << inl.rows << std::endl;
	// cv::solvePnPRefineLM(objw_inl, corners_inl, K, D, rvec, tvec);
	// cv::solvePnPRefineVVS(objw_inl, corners_inl, K, D, rvec, tvec);
	;
	// cv::Matx33d R;

	if (inl.rows < 0.9 * corners.size())
		return false;
	else
	{
		cv::solvePnPGeneric(tmp, corners, K, D, rvecIPPE, tvecIPPE, true, cv::SolvePnPMethod::SOLVEPNP_IPPE);
		Rout = rvecIPPE[0].clone();
		tOut = tvecIPPE[0].clone();
		// cv::solvePnPRefineLM(tmp, corners, K, D, Rout, tOut);
		// cv::solvePnPRefineVVS(tmp, corners, K, D, Rout, tOut);
		return true;
	}
}

void display_resize(const cv::Mat &src, cv::Mat &dst, double scale = 0.75)
{
	cv::resize(src, dst, cv::Size(scale * src.cols, scale * src.rows));
}

void findAprilgridCorners(const cv::Mat &srcGray, std::vector<cv::Point2d> &imagePoints_p2d, std::vector<cv::Point3d> &objectPoints_p3d)
{
	double length = 0.088 * 1000.0;
	double ratio = 0.3;
	calibration_toolkit::AprilgridDetector::AprilgridOptions opt;
	opt.subpixelWindowSize = 5;
	opt.maxSubpixDisplacement2 = 1.0;
	opt.minBorderDistance = 5.0;
	calibration_toolkit::AprilgridDetector detector(length, ratio, opt);
	Eigen::MatrixXd imagePoints_m2d;
	std::vector<bool> imagePoints_flags;
	detector.computeObservation(srcGray, imagePoints_m2d, imagePoints_flags);
	detector.convertResults_OpenCV(imagePoints_m2d, imagePoints_flags, objectPoints_p3d, imagePoints_p2d);
	// cv::calibrateHandEye()
}

bool detect_ours(const cv::Mat &image, cv::Mat K, cv::Mat D,
				 std::vector<Eigen::Matrix2Xd> &views_image_p2ds,
				 std::vector<Eigen::Matrix2Xd> &views_board_p2ds,

				 //  std::vector<Eigen::Matrix2Xd> &views_image_p2ds_norm,
				 //  std::vector<Eigen::Matrix3Xd> &views_board_p3ds,
				 std::vector<cv::Mat> &poses_R_cam_target,
				 std::vector<cv::Mat> &poses_t_cam_target,

				 std::vector<std::vector<cv::Point2f>> &views_image_p2fs_cv,
				 std::vector<std::vector<cv::Point3f>> &views_board_p3fs_cv)
{
	std::vector<cv::Point2d> imagePoints_p2d;
	std::vector<cv::Point3d> objectPoints_p3d;
	findAprilgridCorners(image, imagePoints_p2d, objectPoints_p3d);

	bool detect_result = false;
	bool pose_result = false;
	cv::Mat image_color;

	if (objectPoints_p3d.size() < 4)
		std::cout << "# [INFO] NOT FOUND" << objectPoints_p3d.size() << std::endl;
	else
	{
		detect_result = true;
		Eigen::Matrix2Xd image_p2ds, board_p2ds;
		image_p2ds.resize(2, imagePoints_p2d.size());
		board_p2ds.resize(2, objectPoints_p3d.size());
		image_p2ds.setZero();
		board_p2ds.setZero();
		for (int r = 0; r < imagePoints_p2d.size(); ++r)
		{
			image_p2ds(0, r) = (imagePoints_p2d)[r].x;
			image_p2ds(1, r) = (imagePoints_p2d)[r].y;
			board_p2ds(0, r) = (objectPoints_p3d)[r].x;
			board_p2ds(1, r) = (objectPoints_p3d)[r].y;
		}
		// std::cout << board_p2ds.cols() << std::endl;

		cv::Mat Rout, tout;

		cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
		pose_result = estimateRtFromPlanar(imagePoints_p2d, objectPoints_p3d, K, D, Rout, tout);
		if (pose_result)
			cv::drawFrameAxes(image_color, K, D, Rout, tout, 1000);
		if (detect_result)
			for (const auto &p2d : calibration_toolkit::MatConverter::cvPoint2<float, double>(imagePoints_p2d))
				cv::circle(image_color, p2d, 1, CV_RGB(255, 0, 0), 1);

		if (detect_result && pose_result)
		{

			cv::Mat Rmat;
			cv::Rodrigues(Rout, Rmat);
			poses_R_cam_target.push_back(Rout);
			poses_t_cam_target.push_back(tout);

			views_board_p2ds.push_back(board_p2ds);
			views_image_p2ds.push_back(image_p2ds);

			views_image_p2fs_cv.push_back(calibration_toolkit::MatConverter::cvPoint2<float, double>(imagePoints_p2d));
			views_board_p3fs_cv.push_back(calibration_toolkit::MatConverter::cvPoint3<float, double>(objectPoints_p3d));
		}
		cv::Mat imageDisplay;
		display_resize(image_color, imageDisplay, 0.5);
		cv::imshow("DEBUG", imageDisplay);
		cv::waitKey(1);
	}

	// std::cout << Rout << tout << std::endl;

	return detect_result && pose_result;
}
double calibrate_ours(const std::vector<Eigen::Matrix2Xd> &views_image_p2ds, const std::vector<Eigen::Matrix2Xd> &views_board_p2ds, cv::Mat &K, cv::Mat &D)
{
	Eigen::Matrix3d K_eigen;
	Eigen::VectorXd D_eigen;
	double rmse = calibration_toolkit::calibrateCamera(views_image_p2ds, views_board_p2ds, K_eigen, D_eigen, calibration_toolkit::CalibFlags::CAMERA_K6P2);

	K = calibration_toolkit::MatConverter::matd_cv_eigen<double>(K_eigen).clone();
	D = calibration_toolkit::MatConverter::matd_cv_eigen<double>(D_eigen).clone();
	return rmse;
}
/**
 * @brief https://learnopencv.com/understanding-lens-distortion/
 * @return int
 */
void detect_and_calibrate(cv::String beSaveDataPath, cv::Mat K_input, cv::Mat D_input, cv::Mat &X1_out, cv::Mat &X2_out)
{
	// size_t n_views = 0;
	std::vector<Eigen::Matrix2Xd> views_image_p2ds;
	std::vector<Eigen::Matrix2Xd> views_board_p2ds;
	std::vector<std::vector<cv::Point2f>> views_image_p2fs_cv;
	std::vector<std::vector<cv::Point3f>> views_board_p3fs_cv;
	std::vector<std::vector<cv::Point2f>> views_image_norm_p2fs_cv;
	std::vector<Eigen::Matrix2Xd> views_image_norm_p2ds;
	std::vector<Eigen::Matrix3Xd> views_board_p3ds;
	std::vector<Eigen::Vector2d> thetas;
	std::vector<cv::Mat> poses_R_cam_target;
	std::vector<cv::Mat> poses_t_cam_target;
	std::vector<cv::Mat> poses_R_base_gripper;
	std::vector<cv::Mat> poses_t_base_gripper;
	// const  &thetas,
	// std::string calibrationDir = "/usr/Evo_BionicEyes/io_path/fastSave";
	// std::string beSaveDataPath = "/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE1/motor_wide";
	calibration_toolkit::RecordData_BECSV data(beSaveDataPath, calibration_toolkit::RecordDataHeader_BECSV::jpg);

	Eigen::Matrix4d X1, X2, L;
	// X1.setIdentity();
	// X2.setIdentity();
	// X1.setIdentity();
	// X1.block<3, 3>(0, 0) = Eigen::AngleAxisd(1.92479, Eigen::Vector3d(-0.684465, -0.506321, 0.524544)).toRotationMatrix(); // Yaw
	// X2.block<3, 3>(0, 0) = Eigen::AngleAxisd(1.73774, Eigen::Vector3d(0.363011, -0.854423, 0.371732)).toRotationMatrix();  // Pitch
	X1 << -0.237271437689159, -0.0208968322083928, -0.971218609408084,
		-32.469213215203, 0.971402740863793, -0.0142499514089653,
		-0.237009818208345, 50.653999556616, -0.00888706358870689,
		-0.999680079469675, 0.0236803465404098, 0.598699012346748, 0, 0, 0, 1;
	X1.block<3, 1>(0, 3) = 0 * Eigen::Vector3d::Random();

	X2 << -0.0129208122517066, -0.749981951134545, -0.661332084193093,
		-35.7949207979476, 0.00437531610054809, 0.661338557715894,
		-0.750074775397282, 7.79212334387461, 0.999906950280765,
		-0.0125851122634271, -0.00526362323314267, -0.163265468864144, 0, 0, 0, 1;
	X2.block<3, 1>(0, 3) = 0 * Eigen::Vector3d::Random();
	L << -0.998492687739403, -0.0528576543290368, 0.0147790699901552, 375.343685932303,
		0.0516504683358182, -0.996015565505672, -0.0726995350130025, -119.007330513521,
		0.0185629106454913, -0.0718266082259785, 0.997244381633268, 2809.02038560834,
		0, 0, 0, 1;
	Eigen::Matrix4d X[2];
	X[0].setIdentity();
	X[1].setIdentity();

	calibration_toolkit::KinematicsX1X2 km(&X[0]);

	km.reset();
	std::cout << km.X_SE3[0].transform() << "\n";
	std::cout << km.X_SE3[1].transform() << "\n";

	std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;
	for (long i = data.idMin(); i <= data.idMax(); i = i + 16)
	{
		calibration_toolkit::Frame_BE frame;
		if (data.grabFrame(i, 0, frame, calibration_toolkit::RecordDataHeader_BECSV::ENCODER_DEG, true))
		{
			// std::cout << "# NEW FRAME " << frame.id << " ##############################################################################" << std::endl;
			double theta1 = frame.motorEncoder(5) / 180.0 * CV_PI; // yaw
			double theta2 = frame.motorEncoder(3) / 180.0 * CV_PI; // pitch
			Eigen::VectorXd ts;

			ts.resize(2);
			ts(0) = theta1;
			ts(1) = theta2;
			cv::Mat imageGray;
			cv::Mat tzero;
			tzero.create(3, 1, CV_64FC1);
			tzero.setTo(cv::Scalar(0.0));
			Eigen::Matrix3d R_b_e = km.FK(ts).block<3, 3>(0, 0);
			cv::Mat R_base_gripper = calibration_toolkit::MatConverter::matd_cv_eigen<double>(R_b_e);

			cv::cvtColor(frame.image, imageGray, cv::COLOR_BGR2GRAY);

			if (detect_ours(imageGray, K_input, D_input, views_image_p2ds, views_board_p2ds, poses_R_cam_target, poses_t_cam_target,
							views_image_p2fs_cv, views_board_p3fs_cv))
			{
				std::vector<cv::Point2f> tmp;
				cv::undistortPoints(views_image_p2fs_cv.back(), tmp, K_input, D_input);
				thetas.push_back(Eigen::Vector2d(theta1, theta2));
				views_image_norm_p2fs_cv.push_back(tmp);
				poses_R_base_gripper.push_back(R_base_gripper);
				poses_t_base_gripper.push_back(tzero);
			}
		}
	}

	cv::Mat R_gripper_cam, t_gripper_cam;

	cv::calibrateHandEye(poses_R_base_gripper, poses_t_base_gripper,
						 poses_R_cam_target, poses_t_cam_target, R_gripper_cam, t_gripper_cam,cv::CALIB_HAND_EYE_HORAUD);
	std::cout << R_gripper_cam << std::endl;
	std::cout << t_gripper_cam << std::endl;
	return;
	std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;

	size_t n_views = views_board_p2ds.size();
	views_board_p3ds.resize(n_views);
	views_image_norm_p2ds.resize(n_views);
	for (size_t k = 0; k < n_views; k++)
	{
		// std::cout << views_board_p2ds[k].cols() << std::endl;
		size_t n_detected = views_board_p2ds[k].cols();
		views_board_p3ds[k].resize(3, n_detected);
		views_board_p3ds[k].setZero();
		views_board_p3ds[k].row(0) = views_board_p2ds[k].row(0);
		views_board_p3ds[k].row(1) = views_board_p2ds[k].row(1);
		views_image_norm_p2ds[k].resize(2, n_detected);

		for (size_t b = 0; b < n_detected; b++)
		{
			views_image_norm_p2ds[k](0, b) = views_image_norm_p2fs_cv[k][b].x;
			views_image_norm_p2ds[k](1, b) = views_image_norm_p2fs_cv[k][b].y;
		}
		// views_image_norm_p2fs[k].row(0) = views_board_p2ds[k].row(0);
		// views_image_norm_p2fs[k].row(1) = views_board_p2ds[k].row(1);
	}

	std::cout << "# LANDMARK POSE ###############################################################################" << std::endl;

	calibration_toolkit::kba_X1X2L(views_image_norm_p2ds, views_board_p3ds, thetas, X1, X2, L);
	calibration_toolkit::MatConverter::matd_cv_eigen<double>(X1).copyTo(X1_out);
	calibration_toolkit::MatConverter::matd_cv_eigen<double>(X2).copyTo(X2_out);

	return; /*
	 // L.rotation() = L_guess.block(0, 0, 3, 3);
	 // L.setIdentity();
	 // L.translation(Eigen::Vector3d(L_guess(0, 3), L_guess(1, 3), L_guess(2, 3)));
	 // L.quat(Eigen::Quaterniond(Eigen::Matrix3d(L_guess.block(0, 0, 3, 3))));
	 // std::cout << L.transform() << std::endl;

	 // std::vector<Eigen::Matrix2Xd> views_image_p2ds;
	 // std::vector<Eigen::Matrix2Xd> views_board_p2ds;

	 cv::Mat K_ours, D_ours;
	 // DETECTION
	 //..int count = -1;
	 for (int k = 0; k < imagePaths.size(); k++)
	 {
		 if (k % 100 == 0)
		 {
			 std::string imgPath = imagePaths[k];
			 std::cout << imgPath << std::endl;
			 cv::Mat image = cv::imread(imgPath, cv::IMREAD_GRAYSCALE);
			 detect_ours(image, views_image_p2ds, views_board_p2ds, views_image_p2fs_cv, views_board_p3fs_cv);
		 }
	 }

	 cv::undistortPoints(views_image_p2fs_cv, views_image_norm_p2fs_cv, K_input, D_input);

	 // calibration_toolkit::optimize_X1X2L(views_image_norm_p2fs,views_board_p3ds);

	 // OUR CALIBRATE CAMERA
	 double rmse_ours = calibrate_ours(views_image_p2ds, views_board_p2ds, K_ours, D_ours);

	 std::cout << K_ours << "\n";
	 std::cout << D_ours << "\n";
	 std::cout << rmse_ours << "\n";

	 // OPENCV CALIBRATE CAMERA
	 // cv::Size imageSize = images[0].size();
	 // cv::Mat K_cv, D_cv;
	 // std::vector<cv::Mat> rvecs, tvecs;

	 // double rmse_cv = cv::calibrateCamera(views_board_p3fs_cv, views_image_p2fs_cv, imageSize, K_cv, D_cv, rvecs, tvecs, cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_RATIONAL_MODEL);
	 // rmse_cv = cv::calibrateCamera(views_board_p3fs_cv, views_image_p2fs_cv, imageSize, K_cv, D_cv, rvecs, tvecs, cv::CALIB_FIX_INTRINSIC | cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL);
	 // std::cout << K_cv << "\n";
	 // std::cout << D_cv << "\n";
	 // std::cout << rmse_cv << "\n";

	 // OPENCV UNDISTORTION
	 cv::Mat imageDisplay;
	 cv::Mat K_out = K_ours, D_out = D_ours;
	 for (const auto &imgPath : imagePaths)
	 {
		 cv::Mat img = cv::imread(imgPath, cv::IMREAD_GRAYSCALE);

		 cv::Mat undist;
		 cv::Mat K_new = K_out.clone();
		 cv::Mat D_new = D_out.clone();
		 // K_new = K_new * 0.5;
		 K_new.at<double>(0, 0) *= 0.4;
		 K_new.at<double>(1, 1) *= 0.4;
		 cv::undistort(img, undist, K_ours, D_ours, K_new);
		 display_resize(undist, imageDisplay);
		 cv::imshow("CHECK UNDISTORTION", imageDisplay);
		 cv::waitKey(0);
	 }
	 return;
	 */
}

int main()
{

	calibration_toolkit::MultiCamRig mcr;
	cv::Mat X;
	mcr.readJSON("./ee01.json", "BE310A28210001");
	// mcr.read(mcr.json, "BE1", "K", &(mcr.K[0]), 3, 3);
	std::cout << mcr.K[0] << std::endl;
	// std::cout << mcr.K[1] << std::endl;
	// std::cout << mcr.K[2] << std::endl;
	// std::cout << mcr.K[3] << std::endl;
	std::cout << mcr.D[0] << std::endl;
	// std::cout << mcr.D[1] << std::endl;
	// std::cout << mcr.D[2] << std::endl;
	// mcr.readJSON("./config.json", "BE2");
	// mcr.read(mcr.json, "BE1", "K", &(mcr.K[0]), 3, 3);
	// std::cout << mcr.K[0] << std::endl;
	// std::cout << mcr.K[1] << std::endl;
	// std::cout << mcr.K[2] << std::endl;
	// std::cout << mcr.K[3] << std::endl;
	// std::cout << mcr.X[0] << std::endl;
	// std::cout << mcr.X[1] << std::endl;
	// std::cout << mcr.X[2] << std::endl;
	// mcr.writeJSON("./abc.json", "BE310A28210000_test");
	// mcr.write(mcr.json, "BE1", "K", mcr.K, 3, 3);
	//	return 0;
	// cv::CALIB_CB_ACCURACY
	FLAGS_log_dir = "./";
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging("calibrator");
	google::LogToStderr();
	// std::vector<cv::String> imagesPaths;
	//  cv::glob(
	//  	"/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE0/motor_wide/Cam_0/*.jpg",
	//  	//"/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE0/camera_wide/Cam_0/*.jpg",
	//  	imagesPaths);

	// calibration_toolkit::io::loadFiles("/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE1/motor_wide/Cam_0",
	// 								   ".jpg", imagesPaths);

	// cv::glob("/media/czhou/BE_DATA/DataSets/EVO_BE/EEStereo/EE0/camera_mid/Cam_1/*.jpg", imagesPaths);
	// std::vector<cv::Mat> images;
	// for (const auto &path : images)
	// for (int g = 0; g < imagesPaths.size(); g = g + 5)
	// {
	// 	cv::Mat image = cv::imread(imagesPaths[g], cv::IMREAD_GRAYSCALE);
	// 	images.push_back(image);
	// }
	// cv::Mat K, D;
	// cv::Mat K_input = cv::Mat(3, 3, CV_64FC1, K_data);
	// // cv::Mat H_RL_mma = cv::Mat(3, 3, CV_64FC1, H_RL_data);
	// cv::Mat D_input = cv::Mat(8, 1, CV_64FC1, D_data);
	detect_and_calibrate(std::string("/media/czhou/DATA/EE/Indoor_00/1_calib/motor_wide"), mcr.K[0], mcr.D[0], mcr.X[0], mcr.X[1]);
	mcr.writeJSON("./ee1.json", "BE310A28210001");
}