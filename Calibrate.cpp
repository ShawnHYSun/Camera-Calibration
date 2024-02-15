// Haoyuan Sun
// haoyuansun712@gmail.com

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace cv;
using namespace std;
using namespace Eigen;


// The number of corner points and the spacing of the checkerboard
// These parameters are subject to personal actual conditions
const int XX = 8;
const int YY = 8;
const int L = 5;
const int width = 960;
const int height = 540;

Eigen::MatrixXd reorientate_corners(const cv::Mat& image, const Eigen::MatrixXd& corners, int size, const std::vector<int>& positive_inds) {
	// Find all circles
	cv::SimpleBlobDetector::Params params;
	params.blobColor = 0;
	params.filterByColor = true;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	vector<cv::KeyPoint> keypoints;
	detector->detect(image, keypoints);

	Eigen::Map<Eigen::MatrixXd> corners_(const_cast<double*>(corners.data()), size * size, corners.cols());

	for (int rot = 0; rot < 4; rot++) {
		Map<MatrixXd> corners_mat(corners_.data(), size, size);

		switch (rot) {
		case 1:
			corners_mat = corners_mat.transpose().colwise().reverse();
			break;
		case 2:
			corners_mat = corners_mat.colwise().reverse().rowwise().reverse();
			break;
		case 3:
			corners_mat = corners_mat.transpose().rowwise().reverse();
			break;
		}

		vector<vector<Point2f>> white_bbs;

		for (int ind = 0; ind < (size - 1) * (size - 1) - 1; ind++) {
			if (ind % 2 == 1) {
				vector<Point2f> bb(4);
				bb[0] = cv::Point2f(corners_mat(static_cast<int>(ind + ind / (size - 1)), 0), corners_mat(static_cast<int>(ind + ind / (size - 1)), 1));
				bb[1] = cv::Point2f(corners_mat(static_cast<int>(ind + ind / (size - 1) + 1), 0), corners_mat(static_cast<int>(ind + ind / (size - 1) + 1), 1));
				bb[2] = cv::Point2f(corners_mat(static_cast<int>(ind + ind / (size - 1) + size + 1), 0), corners_mat(static_cast<int>(ind + ind / (size - 1) + size + 1), 1));
				bb[3] = cv::Point2f(corners_mat(static_cast<int>(ind + ind / (size - 1) + size), 0), corners_mat(static_cast<int>(ind + ind / (size - 1) + size), 1));

				white_bbs.push_back(bb);
			}
		}

		vector<cv::KeyPoint> valid_circles;
		vector<int> valid_indexes;
		for (size_t i = 0; i < keypoints.size(); i++) {
			for (int ind = 0; ind < white_bbs.size(); ind++) {
				if (cv::pointPolygonTest(white_bbs[ind], keypoints[i].pt, false) > 0) {
					valid_circles.push_back(keypoints[i]);
					valid_indexes.push_back(ind);
					break;
				}
			}
		}

		if (valid_circles.size() == positive_inds.size()) {
			set<int> valid_set(valid_indexes.begin(), valid_indexes.end());
			set<int> positive_set(positive_inds.begin(), positive_inds.end());

			if (valid_set == positive_set) {
				return corners_;
			}
		}
	}

	return Eigen::MatrixXd();;
}


Matrix4d read_tfm_from_csv(const std::string &path, const std::string &tx_label, const std::string &ty_label, const std::string &tz_label,
	const std::string &qx_label, const std::string &qy_label, const std::string &qz_label, const std::string &qw_label) {
	std::ifstream file(path);
	std::vector<double> tx_data, ty_data, tz_data, qx_data, qy_data, qz_data, qw_data;

	// Read the data in the CSV file and store it in the corresponding variables
	while (!file.eof()) {
		double tx, ty, tz, qx, qy, qz, qw;
		file >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
		tx_data.push_back(tx);
		ty_data.push_back(ty);
		tz_data.push_back(tz);
		qx_data.push_back(qx);
		qy_data.push_back(qy);
		qz_data.push_back(qz);
		qw_data.push_back(qw);
	}

	// Calculate the average number
	double tx = std::accumulate(tx_data.begin(), tx_data.end(), 0.0) / tx_data.size();
	double ty = std::accumulate(ty_data.begin(), ty_data.end(), 0.0) / ty_data.size();
	double tz = std::accumulate(tz_data.begin(), tz_data.end(), 0.0) / tz_data.size();
	double qx = std::accumulate(qx_data.begin(), qx_data.end(), 0.0) / qx_data.size();
	double qy = std::accumulate(qy_data.begin(), qy_data.end(), 0.0) / qy_data.size();
	double qz = std::accumulate(qz_data.begin(), qz_data.end(), 0.0) / qz_data.size();
	double qw = std::accumulate(qw_data.begin(), qw_data.end(), 0.0) / qw_data.size();

	Eigen::Quaterniond quaternion(qw, qx, qy, qz);
	Eigen::Matrix3d rotation_matrix = quaternion.normalized().toRotationMatrix();

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
	transformation_matrix.block<3, 1>(0, 3) << tx, ty, tz;

	return transformation_matrix;
}

	
struct CalibrationResult {
	cv::Mat mtx;
	cv::Mat dist;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	std::vector<std::vector<cv::Point3f>> obj_points;
};




CalibrationResult calibration(const std::vector<std::string>& paths) {
	CalibrationResult result;

	std::vector<std::vector<cv::Point2f>> img_points;  // Use Point2f here
	std::vector<std::vector<cv::Point3f>> obj_points;

	Eigen::MatrixXf objp(XX * YY, 3);
	objp.col(0) = Eigen::ArrayXXf::LinSpaced(XX, 0, XX - 1).replicate(YY, 1).matrix();
	objp.col(1) = Eigen::ArrayXXf::LinSpaced(YY, 0, YY - 1).transpose().replicate(1, XX).matrix();
	objp.col(2).setZero();
	Eigen::Matrix3f L;
	objp = L * objp;

	cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001);

	// Traverse image path
	for (const auto& path : paths) {
		cv::Mat img = cv::imread(path);
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

		cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

		cv::Size size = gray.size();
		cv::Size patternSize(XX, YY);
		std::vector<cv::Point2f> corners;

		bool ret = cv::findChessboardCorners(gray, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH);

		if (ret) {
			// Convert Eigen::MatrixXf to std::vector<cv::Point3f>
			std::vector<cv::Point3f> obj_points_vec;
			for (int i = 0; i < objp.rows(); ++i) {
				obj_points_vec.push_back(cv::Point3f(objp(i, 0), objp(i, 1), objp(i, 2)));
			}

			obj_points.push_back(obj_points_vec);

			// Find sub-pixel corner points based on original corner points
		    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

			todo: bug 1
			Eigen::MatrixXd reorientedCorners = reorientate_corners(
				gray,
				Eigen::Map<Eigen::MatrixXd>(corners.data, corners.rows, corners.cols),
				8,
				{ 7, 9, 14 }
			);

			img_points.push_back(corners);

		}
	}


	// Conduct camera calibration
	cv::Mat mtx, dist;
	std::vector<cv::Mat> rvecs, tvecs;

	todo: bug2
	cv::calibrateCamera(obj_points, img_points, size, mtx, dist, rvecs, tvecs);

	result.mtx = mtx;
	result.dist = dist;
	result.rvecs = rvecs;
	result.tvecs = tvecs;
	result.obj_points = obj_points;

	return result;
}



void undistort_images(const std::vector<std::string>& paths, const std::string& new_folder, cv::Mat& mtx, cv::Mat& dist) {
	
	std::vector<std::string> sortedPaths = paths;
	std::sort(sortedPaths.begin(), sortedPaths.end());

	for (const std::string& path : sortedPaths) {
		cv::Mat figure = cv::imread(path);
		cv::Mat dst;
		cv::undistort(figure, dst, mtx, dist);

		cv::imwrite(new_folder + "/" + cv::format("%s", cv::samples::findFile(path)), dst);
	}
}


Eigen::Matrix4d read_tfm_from_csv(const std::string& pose_path) {
	Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

	std::ifstream file(pose_path);
	if (file.is_open()) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				file >> pose(i, j);
			}
		}
		file.close();
	}

	return pose;
}


std::pair<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>> get_poses(
	const std::vector<std::string>& image_paths, const std::string& pose_folder) {

	std::vector<std::string> pose_paths;

	for (const std::string& item : image_paths) {
		std::string pose_path = pose_folder + "/" + item.substr(item.find_last_of('/') + 1, item.size() - item.find_last_of('/') - 5) + ".csv";
		if (std::ifstream(pose_path)) {
			pose_paths.push_back(pose_path);
		}
	}

	if (pose_paths.size() != image_paths.size()) {
		exit(-1);
	}

	std::vector<Eigen::Matrix4d> poses;
	for (const std::string& pose_path : pose_paths) {
		poses.push_back(read_tfm_from_csv(pose_path));
	}

	std::vector<Eigen::Matrix3d> Rs;
	std::vector<Eigen::Vector3d> ts;

	for (const Eigen::Matrix4d& pose : poses) {
		Rs.push_back(pose.block<3, 3>(0, 0));
		ts.push_back(pose.block<3, 1>(0, 3));
	}

	return std::make_pair(Rs, ts);
}


Eigen::Matrix3d calibrate_handeye(const std::vector<Eigen::Matrix3d>& Rs_tool2base, const std::vector<Eigen::Vector3d>& ts_tool2base, const std::vector<Eigen::Matrix3d>& Rs_target2cam, const std::vector<Eigen::Vector3d>& ts_target2cam) {
	std::vector<Eigen::Matrix3d> Rs_cam2tool;
	std::vector<Eigen::Vector3d> ts_cam2tool;

	for (int i = 0; i < 5; i++) {
		Eigen::Matrix3d R_cam2tool;
		Eigen::Vector3d t_cam2tool;
		// Implement calibration using method i
		// R_cam2tool and t_cam2tool should be computed here

		// Calculate T_cam2tool
		Eigen::Matrix4d T_cam2tool = Eigen::Matrix4d::Identity();
		T_cam2tool.block<3, 3>(0, 0) = R_cam2tool;
		T_cam2tool.block<3, 1>(0, 3) = t_cam2tool;

		for (int ind = 0; ind < Rs_tool2base.size(); ind++) {
			Eigen::Matrix4d T_target2cam = Eigen::Matrix4d::Identity();
			T_target2cam.block<3, 3>(0, 0) = Rs_target2cam[ind];
			T_target2cam.block<3, 1>(0, 3) = ts_target2cam[ind];

			Eigen::Matrix4d T_tool2base = Eigen::Matrix4d::Identity();
			T_tool2base.block<3, 3>(0, 0) = Rs_tool2base[ind];
			T_tool2base.block<3, 1>(0, 3) = ts_tool2base[ind];

			Eigen::Matrix4d T_target2base = T_tool2base * T_cam2tool * T_target2cam;

			// Process T_target2base
		}

		// Update Rs_cam2tool and ts_cam2tool
		Rs_cam2tool.push_back(R_cam2tool);
		ts_cam2tool.push_back(t_cam2tool);
	}

	// Calculate the mean R_cam2tool and t_cam2tool
	Eigen::Matrix3d R_cam2tool_mean = Eigen::Matrix3d::Zero();
	Eigen::Vector3d t_cam2tool_mean = Eigen::Vector3d::Zero();

	for (const Eigen::Matrix3d& R_cam2tool : Rs_cam2tool) {
		R_cam2tool_mean += R_cam2tool;
	}
	R_cam2tool_mean /= Rs_cam2tool.size();

	for (const Eigen::Vector3d& t_cam2tool : ts_cam2tool) {
		t_cam2tool_mean += t_cam2tool;
	}
	t_cam2tool_mean /= ts_cam2tool.size();

	return R_cam2tool_mean, t_cam2tool_mean;
}

std::vector<Eigen::Vector3d> vertex_projection(const Eigen::Matrix3d& mtx, double shift) {
	Eigen::Matrix3d Lcam = mtx;
	Lcam.block<1, 2>(0, 2) = Eigen::Vector2d::Zero();

	std::vector<Eigen::Vector3d> control_points = { Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(width, 0, 1), Eigen::Vector3d(width, height, 1), Eigen::Vector3d(0, height, 1) };
	std::vector<Eigen::Vector3d> projected_points;

	for (const Eigen::Vector3d& p : control_points) {
		Eigen::Vector3d projected_point = Lcam.inverse() * (-shift * Lcam.col(2) - Lcam.col(3));
		projected_points.push_back(projected_point);
	}

	return projected_points;
}


int main() {
	std::string image_folder = "Input your own raw image folder's path";
	std::string undist_folder = "C:\\undistorted_images";
	std::string pose_folder = "Input your own pose CSV files folder's path";

	std::vector<std::string> image_paths;

	Eigen::Matrix3d mtx, dist;
	std::vector<Eigen::Matrix3d> Rs_target2cam;
	std::vector<Eigen::Vector3d> ts_target2cam;
	std::vector<Eigen::Vector3d> obj_points;
	std::vector<Eigen::Vector2d> img_points;

	Eigen::Matrix3d mtx_undist, dist_undist;
	std::vector<Eigen::Matrix3d> Rs_tool2base;
	std::vector<Eigen::Vector3d> ts_tool2base;

	Eigen::Matrix3d R_cam2tool;
	Eigen::Vector3d t_cam2tool;

	Eigen::Matrix4d T_cam2tool = Eigen::Matrix4d::Identity();
	T_cam2tool.block<3, 3>(0, 0) = R_cam2tool;
	T_cam2tool.block<3, 1>(0, 3) = t_cam2tool;

	std::vector<Eigen::Vector3d> projected_vertexes_cam;
	double shift = 20.0;
	std::vector<Eigen::Vector3d> projected_vertexes_tool;

	for (const Eigen::Vector3d& projected_vertex_cam : projected_vertexes_cam) {
		Eigen::Vector4d projected_vertex_cam_4d(projected_vertex_cam.x(), projected_vertex_cam.y(), projected_vertex_cam.z(), 1.0);
		Eigen::Vector4d projected_vertex_tool_4d = T_cam2tool * projected_vertex_cam_4d;
		projected_vertexes_tool.push_back(projected_vertex_tool_4d.block<3, 1>(0, 0));
	}

	// Print or use the projected_vertexes_tool
	return 0;
}
