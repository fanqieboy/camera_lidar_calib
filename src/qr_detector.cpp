/**
 * @file qr_detector.cpp
 * @brief QR/ArUco检测模块实现文件

 *
 * 该文件实现了独立的QR/ArUco标记检测器，完全移除ROS依赖。
 */

#include "qr_detector.h"

#include "common_lib.h"
#include "config_manager.h"

#include <iostream>
#include <algorithm>
#include <cassert>
#include <cmath>

QRDetector::QRDetector(const Params& params) : verbose_(true), total_detections_(0), successful_detections_(0) {
  // 初始化相机参数
  InitializeCameraParameters(params);

  // 保存相机ID
  camera_id_ = params.camera_id;

  // 创建ArUco字典
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  // 创建标定板配置
  CreateBoardConfiguration();

  // 设置默认检测参数
  parameters_ = cv::aruco::DetectorParameters::create();
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
  parameters_->doCornerRefinement = true;
#else
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#endif

  if (verbose_) {
    std::cout << "[QRDetector] 初始化完成" << std::endl;
    std::cout << "  标记尺寸: " << marker_size_ << "m" << std::endl;
    std::cout << "  最小检测标记数: " << min_detected_markers_ << std::endl;
  }
}

QRDetector::~QRDetector() = default;

void QRDetector::InitializeCameraParameters(const Params& params) {
  // 从参数中提取标定板相关尺寸
  marker_size_ = params.marker_size;
  delta_width_qr_center_ = params.delta_width_qr_center;
  delta_height_qr_center_ = params.delta_height_qr_center;
  delta_width_circles_ = params.delta_width_circles;
  delta_height_circles_ = params.delta_height_circles;
  min_detected_markers_ = params.min_detected_markers;

  // 保存相机ID
  camera_id_ = params.camera_id;

  // 创建相机内参矩阵
  camera_matrix_ = (cv::Mat_<double>(3, 3) << params.fx, 0, params.cx, 0, params.fy, params.cy, 0, 0, 1);

  // 创建畸变系数
  // 添加新的逻辑:
  if (!params.distortion_coeffs.empty()) {
    // 从vector直接创建Mat
    distortion_coeffs_ = cv::Mat(params.distortion_coeffs, true).clone();
  } else {
    // 如果没有畸变系数，则创建一个全零的
    distortion_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  }
}

void QRDetector::CreateBoardConfiguration() {
  // 清空之前的配置
  board_corners_.clear();
  board_circle_centers_.clear();
  // 设置标定板标记ID
  board_ids_ = {1, 2, 4, 3};

  // 根据参数计算标定板几何尺寸
  float width = delta_width_qr_center_;
  float height = delta_height_qr_center_;
  float circle_width = delta_width_circles_ / 2.0;
  float circle_height = delta_height_circles_ / 2.0;

  board_corners_.resize(4);

  // 为每个角落的标记生成几何配置
  for (int i = 0; i < 4; ++i) {
    // 计算当前标记中心的坐标
    int x_qr_center = (i % 3) == 0 ? -1 : 1;
    int y_qr_center = (i < 2) ? 1 : -1;
    float x_center = x_qr_center * width;
    float y_center = y_qr_center * height;

    // 添加圆心
    cv::Point3f circle_center(x_qr_center * circle_width, y_qr_center * circle_height, 0);
    board_circle_centers_.push_back(circle_center);

    // 添加标记角点
    for (int j = 0; j < 4; ++j) {
      int x_qr = (j % 3) == 0 ? -1 : 1;
      int y_qr = (j < 2) ? 1 : -1;
      cv::Point3f pt3d(x_center + x_qr * marker_size_ / 2.0, y_center + y_qr * marker_size_ / 2.0, 0);
      board_corners_[i].push_back(pt3d);
    }
  }

  // 创建标定板对象
  board_ = cv::aruco::Board::create(board_corners_, dictionary_, board_ids_);
}

QRDetectionResult QRDetector::DetectMarkers(const cv::Mat& image) {
  QRDetectionResult result;
  total_detections_++;

  if (image.empty()) {
    result.error_message = "输入图像为空";
    last_error_ = result.error_message;
    return result;
  }

  // 复制图像用于标注
  image.copyTo(processed_image_);

  try {
    // 检测ArUco标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters_);

    result.marker_ids = ids;
    result.marker_corners = corners;
    result.markers_detected = ids.size();

    if (ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(processed_image_, corners, ids);
    }

    if (ids.size() < min_detected_markers_) {
      result.error_message =
          "检测到的标记数量不足: " + std::to_string(ids.size()) + "/" + std::to_string(min_detected_markers_);
      if (verbose_) {
        std::cout << "[QRDetector] " << result.error_message << std::endl;
      }
      return result;
    }

    if (ids.size() > kTargetNumCircles) {
      result.error_message =
          "检测到的标记数量过多: " + std::to_string(ids.size()) + "/" + std::to_string(kTargetNumCircles);
      if (verbose_) {
        std::cout << "[QRDetector] " << result.error_message << std::endl;
      }
      return result;
    }

    // 估计标定板位姿
    if (!EstimateBoardPose(corners, ids, result.board_rvec, result.board_tvec)) {
      result.error_message = "标定板位姿估计失败";
      return result;
    }

    // 绘制坐标轴
    cv::aruco::drawAxis(processed_image_, camera_matrix_, distortion_coeffs_, result.board_rvec, result.board_tvec,
                        0.2);

    // 计算圆心3D坐标
    cv::Mat R;
    cv::Rodrigues(result.board_rvec, R);

    cv::Mat t = (cv::Mat_<double>(3, 1) << result.board_tvec[0], result.board_tvec[1], result.board_tvec[2]);

    cv::Mat board_transform = cv::Mat::eye(3, 4, CV_64F);
    R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
    t.copyTo(board_transform.rowRange(0, 3).col(3));

    pcl::PointCloud<pcl::PointXYZI>::Ptr candidates_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (size_t i = 0; i < board_circle_centers_.size(); ++i) {
      cv::Mat mat = (cv::Mat_<double>(4, 1) << board_circle_centers_[i].x, board_circle_centers_[i].y,
                     board_circle_centers_[i].z, 1.0);

      cv::Mat mat_qr = board_transform * mat;
      cv::Point3f center3d(mat_qr.at<double>(0, 0), mat_qr.at<double>(1, 0), mat_qr.at<double>(2, 0));

      // 投影到图像并绘制
      cv::Point2f uv = ProjectPointWithDistortion(center3d, camera_matrix_, distortion_coeffs_);
      cv::circle(processed_image_, uv, 5, cv::Scalar(0, 255, 0), -1);

      // 添加到候选点云
      pcl::PointXYZI qr_center;
      qr_center.x = center3d.x;
      qr_center.y = center3d.y;
      qr_center.z = center3d.z;
      qr_center.intensity = 0.0;
      candidates_cloud->push_back(qr_center);
    }

    // 生成组合并验证几何形状
    std::vector<std::vector<int>> groups;
    GenerateCombinations(candidates_cloud->size(), kTargetNumCircles, groups);

    if (groups.empty()) {
      result.error_message = "无法生成有效的候选组合";
      return result;
    }

    int best_candidate_idx = -1;
    double best_candidate_score = -1.0;

    for (size_t i = 0; i < groups.size(); ++i) {
      std::vector<pcl::PointXYZI> candidates;
      for (int j : groups[i]) {
        candidates.push_back(candidates_cloud->at(j));
      }

      double score = ValidateGeometry(candidates);

      if (score > best_candidate_score) {
        best_candidate_score = score;
        best_candidate_idx = i;
      }
    }

    if (best_candidate_idx == -1 || best_candidate_score <= 0) {
      result.error_message = "无法找到符合几何形状的候选组合";
      return result;
    }
    // 1. 先将未排序的最佳候选点存入一个临时点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr unsorted_centers(new pcl::PointCloud<pcl::PointXYZI>());
    for (int j : groups[best_candidate_idx]) {
      unsorted_centers->push_back(candidates_cloud->at(j));
    }

    // 2. 调用我们自己的排序函数，使用存储的 camera_id_
    // 注意：排序是在相机坐标系下进行的，所以最后一个参数是 "camera"
    ::SortPatternCenters(unsorted_centers, result.circle_centers, "camera");

    // 绘制最终选择的圆心
    if (verbose_) {
      for (size_t i = 0; i < result.circle_centers->size(); i++) {
        cv::Point3f pt_circle(result.circle_centers->at(i).x, result.circle_centers->at(i).y,
                              result.circle_centers->at(i).z);
        cv::Point2f uv_circle = ProjectPointWithDistortion(pt_circle, camera_matrix_, distortion_coeffs_);
        cv::circle(processed_image_, uv_circle, 2, cv::Scalar(255, 0, 255), -1);
      }
    }

    result.success = true;
    result.detection_confidence = best_candidate_score;
    successful_detections_++;

    if (verbose_) {
      std::cout << "[QRDetector] 检测成功: " << result.circle_centers->size()
                << " 个圆心, 置信度: " << result.detection_confidence << std::endl;
    }
  } catch (const cv::Exception& e) {
    result.error_message = "OpenCV 错误: " + std::string(e.what());
    last_error_ = result.error_message;
  } catch (const std::exception& e) {
    result.error_message = "处理错误: " + std::string(e.what());
    last_error_ = result.error_message;
  }

  return result;
}

bool QRDetector::DetectQR(const cv::Mat& image, pcl::PointCloud<pcl::PointXYZI>::Ptr centers_cloud) {
  QRDetectionResult result = DetectMarkers(image);

  if (result.success && result.circle_centers->size() == kTargetNumCircles) {
    *centers_cloud = *result.circle_centers;
    return true;
  }

  last_error_ = result.error_message;
  return false;
}

cv::Mat QRDetector::GetProcessedImage() const { return processed_image_.clone(); }

cv::Mat QRDetector::GetCameraMatrix() const { return camera_matrix_.clone(); }

cv::Mat QRDetector::GetDistortionCoeffs() const { return distortion_coeffs_.clone(); }

void QRDetector::UpdateParameters(const Params& params) {
  InitializeCameraParameters(params);
  CreateBoardConfiguration();
}

void QRDetector::SetVerbose(bool verbose) { verbose_ = verbose; }

void QRDetector::SetDetectionParameters(bool corner_refinement, int adaptive_thresh_win_size_min,
                                        int adaptive_thresh_win_size_max) {
  if (parameters_) {
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    parameters_->doCornerRefinement = corner_refinement;
#else
    if (corner_refinement) {
      parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    } else {
      parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif
    parameters_->adaptiveThreshWinSizeMin = adaptive_thresh_win_size_min;
    parameters_->adaptiveThreshWinSizeMax = adaptive_thresh_win_size_max;
  }
}

bool QRDetector::SaveProcessedImage(const std::string& filepath) const {
  if (processed_image_.empty()) {
    return false;
  }

  try {
    return cv::imwrite(filepath, processed_image_);
  } catch (const cv::Exception& e) {
    return false;
  }
}

std::string QRDetector::GetLastError() const { return last_error_; }

std::string QRDetector::GetDetectionStats() const {
  return "检测统计 - 总计: " + std::to_string(total_detections_) + ", 成功: " + std::to_string(successful_detections_) +
         " (" + std::to_string(total_detections_ > 0 ? (successful_detections_ * 100 / total_detections_) : 0) + "%)";
}

cv::Point2f QRDetector::ProjectPointWithDistortion(const cv::Point3f& pt_cv, const cv::Mat& intrinsics,
                                                   const cv::Mat& dist_coeffs) const {
  std::vector<cv::Point3f> input{pt_cv};
  std::vector<cv::Point2f> projected_points;
  projected_points.resize(1);

  // 检查是否为鱼眼模型 (通常鱼眼模型提供 4 个畸变系数: k1, k2, k3, k4)
  if (dist_coeffs.rows * dist_coeffs.cols == 4) {
    cv::fisheye::projectPoints(input, projected_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                               intrinsics, dist_coeffs);
  } else {
    // 标准针孔模型模型 (k1, k2, p1, p2, k3...)
    cv::projectPoints(input, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), intrinsics, dist_coeffs,
                      projected_points);
  }

  return projected_points[0];
}

void QRDetector::GenerateCombinations(int n, int k, std::vector<std::vector<int>>& groups) const {
  groups.clear();

  if (k > n || k <= 0) return;

  std::string bitmask(k, 1);
  bitmask.resize(n, 0);

  do {
    std::vector<int> group;
    for (int i = 0; i < n; ++i) {
      if (bitmask[i]) {
        group.push_back(i);
      }
    }
    groups.push_back(group);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  if (verbose_ && !groups.empty()) {
    std::cout << "[QRDetector] " << n << " 个候选点生成 " << groups.size() << " 种组合" << std::endl;
  }
}

double QRDetector::ValidateGeometry(const std::vector<pcl::PointXYZI>& candidates) const {
  if (candidates.size() != kTargetNumCircles) {
    return -1.0;
  }

  // 计算中心点
  pcl::PointXYZI center;
  center.x = center.y = center.z = 0;
  for (const auto& pt : candidates) {
    center.x += pt.x;
    center.y += pt.y;
    center.z += pt.z;
  }
  center.x /= candidates.size();
  center.y /= candidates.size();
  center.z /= candidates.size();

  // 检查点到中心的距离是否合理
  double expected_diagonal =
      sqrt(delta_width_circles_ * delta_width_circles_ + delta_height_circles_ * delta_height_circles_) / 2.0;

  for (const auto& pt : candidates) {
    double distance = sqrt((pt.x - center.x) * (pt.x - center.x) + (pt.y - center.y) * (pt.y - center.y) +
                           (pt.z - center.z) * (pt.z - center.z));

    if (std::abs(distance - expected_diagonal) / expected_diagonal > kGeometryTolerance) {
      return 0.0;  // 几何验证失败
    }
  }

  return 1.0;  // 几何验证成功
}

bool QRDetector::EstimateBoardPose(const std::vector<std::vector<cv::Point2f>>& corners, const std::vector<int>& ids,
                                   cv::Vec3d& rvec, cv::Vec3d& tvec) const {
  try {
    // 处理鱼眼相机 (4 个参数) 的高精度位姿估计
    if (distortion_coeffs_.total() == 4) {
      std::vector<cv::Point3f> obj_pts_all;
      std::vector<cv::Point2f> image_pts_all;
      
      // 收集所有检测到的角点的 3D（标定板系）和 2D（图像系）对应关系
      for (size_t i = 0; i < ids.size(); i++) {
        int id = ids[i];
        // 查找对应的标定板角点 3D 坐标
        auto it = std::find(board_ids_.begin(), board_ids_.end(), id);
        if (it != board_ids_.end()) {
          int idx = std::distance(board_ids_.begin(), it);
          // 4个角点
          for (int j = 0; j < 4; j++) {
            obj_pts_all.push_back(board_corners_[idx][j]);
            image_pts_all.push_back(corners[i][j]);
          }
        }
      }

      if (image_pts_all.empty()) return false;

      // --- 核心修复: 对鱼眼角点进行预先去畸变 ---
      std::vector<cv::Point2f> undistorted_pts;
      cv::fisheye::undistortPoints(image_pts_all, undistorted_pts, camera_matrix_, distortion_coeffs_);
      
      // 注意: undistortedPoints 输出的是归一化坐标 (cx=0, cy=0, f=1)
      // 使用 solvePnP 配合单位矩阵解算
      cv::Mat rvec_mat, tvec_mat;
      bool success = cv::solvePnP(obj_pts_all, undistorted_pts, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rvec_mat, tvec_mat);
      
      if (success) {
        rvec = rvec_mat;
        tvec = tvec_mat;
        return true;
      }
      return false;
    }

    // --- 标准逻辑 (用于非鱼眼相机) ---
    // 先估计单个标记的位姿并平均
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, distortion_coeffs_, rvecs, tvecs);

    cv::Vec3f rvec_sin(0, 0, 0), rvec_cos(0, 0, 0);
    tvec = cv::Vec3d(0, 0, 0);

    for (size_t i = 0; i < ids.size(); i++) {
      tvec[0] += tvecs[i][0];
      tvec[1] += tvecs[i][1];
      tvec[2] += tvecs[i][2];
      rvec_sin[0] += sin(rvecs[i][0]);
      rvec_sin[1] += sin(rvecs[i][1]);
      rvec_sin[2] += sin(rvecs[i][2]);
      rvec_cos[0] += cos(rvecs[i][0]);
      rvec_cos[1] += cos(rvecs[i][1]);
      rvec_cos[2] += cos(rvecs[i][2]);
    }

    tvec = tvec / double(ids.size());
    rvec_sin = rvec_sin / float(ids.size());
    rvec_cos = rvec_cos / float(ids.size());
    rvec[0] = atan2(rvec_sin[0], rvec_cos[0]);
    rvec[1] = atan2(rvec_sin[1], rvec_cos[1]);
    rvec[2] = atan2(rvec_sin[2], rvec_cos[2]);

    // 使用标定板整体位姿估计进行精化
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board_, camera_matrix_, distortion_coeffs_, rvec, tvec);
#else
    int valid =
        cv::aruco::estimatePoseBoard(corners, ids, board_, camera_matrix_, distortion_coeffs_, rvec, tvec, true);
#endif

    return valid > 0;
  } catch (const cv::Exception& e) {
    last_error_ = "位姿估计错误: " + std::string(e.what());
    return false;
  }
}
