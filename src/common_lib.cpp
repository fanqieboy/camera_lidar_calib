/**
 * @file common_lib.cpp
 * @brief 公共工具函数库实现文件
 *
 * 该文件实现了 项目中的公共工具函数，包括点云处理、图像投影、
 * 数据保存等功能。提供标定流程中通用的算法工具实现。
 */

#include "common_lib.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <pcl/common/centroid.h>

#include "color.h"
#include "qr_detector.h"

// 【新增】SortPatternCenters 的全局实现
void SortPatternCenters(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, pcl::PointCloud<pcl::PointXYZI>::Ptr v,
                        const std::string& input_coord_frame) {
  if (pc->size() != 4) {
    std::cerr << "错误: [SortPatternCenters] 输入的点中心数量必须为4。" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr work_pc(new pcl::PointCloud<pcl::PointXYZI>());
  work_pc->reserve(4);

  // 1. 【核心】根据相机ID，选择不同的坐标系转换 (仅当输入是雷达坐标系时)
  bool needs_transform = (input_coord_frame == "lidar");
  if (needs_transform) {
    for (const auto& p : *pc) {
      pcl::PointXYZI pt;

      // 默认使用camera_0 (雷达前，相机前)
      float cam_x = -p.y;  // LiDAR Y (左) -> Cam -X (右)
      float cam_y = -p.z;  // LiDAR Z (上) -> Cam -Y (上)
      float cam_z = p.x;   // LiDAR X (前) -> Cam  Z (前)

      pt.x = cam_x;
      pt.y = cam_y;
      pt.z = cam_z;
      pt.intensity = p.intensity;
      work_pc->push_back(pt);
    }
  } else {
    // 如果输入已经是相机坐标系，则直接复制
    *work_pc = *pc;
  }

  // 2. 在(临时的)相机坐标系下进行排序
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*work_pc, centroid);
  pcl::PointXYZ ref_origin(centroid[0], centroid[1], centroid[2]);

  std::vector<std::pair<float, int>> proj_points;
  for (size_t i = 0; i < work_pc->size(); ++i) {
    const auto& p = work_pc->points[i];
    proj_points.emplace_back(atan2(p.y - ref_origin.y, p.x - ref_origin.x), static_cast<int>(i));
  }

  std::sort(proj_points.begin(), proj_points.end());

  v->resize(4);
  for (int i = 0; i < 4; ++i) {
    (*v)[i] = work_pc->points[proj_points[i].second];
  }

  const auto& p0 = v->points[0];
  const auto& p1 = v->points[1];
  const auto& p2 = v->points[2];
  Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
  Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);
  if (v01.cross(v12).z() > 0) {
    std::swap((*v)[1], (*v)[3]);
  }

  // 3. 【核心】如果之前转换过，现在需要转换回去，确保输出点云在原始坐标系下
  if (needs_transform) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_sorted_pc(new pcl::PointCloud<pcl::PointXYZI>());
    final_sorted_pc->reserve(4);
    for (const auto& point : v->points) {
      pcl::PointXYZI original_pt;
      // 默认使用camera_0的逆变换
      float lidar_x = point.z;
      float lidar_y = -point.x;
      float lidar_z = -point.y;

      original_pt.x = lidar_x;
      original_pt.y = lidar_y;
      original_pt.z = lidar_z;
      original_pt.intensity = point.intensity;
      final_sorted_pc->push_back(original_pt);
    }
    *v = *final_sorted_pc;  // 将最终结果复制回输出点云
  }
}

double ComputeRmse(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2) {
  if (cloud1->size() != cloud2->size()) {
    std::cerr << COLOR_BOLD_RED << "[ComputeRmse] Point cloud sizes do not match, cannot compute RMSE." << COLOR_RESET
              << std::endl;
    return -1.0;
  }

  double sum = 0.0;
  for (size_t i = 0; i < cloud1->size(); ++i) {
    double dx = cloud1->points[i].x - cloud2->points[i].x;
    double dy = cloud1->points[i].y - cloud2->points[i].y;
    double dz = cloud1->points[i].z - cloud2->points[i].z;

    sum += dx * dx + dy * dy + dz * dz;
  }

  double mse = sum / cloud1->size();
  return std::sqrt(mse);
}

void AlignPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud, const Eigen::Matrix4f& transformation) {
  output_cloud->clear();
  for (const auto& pt : input_cloud->points) {
    Eigen::Vector4f pt_homogeneous(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4f transformed_pt = transformation * pt_homogeneous;

    pcl::PointXYZI output_pt;
    output_pt.x = transformed_pt(0);
    output_pt.y = transformed_pt(1);
    output_pt.z = transformed_pt(2);
    output_pt.intensity = pt.intensity;
    output_cloud->push_back(output_pt);
  }
}

void ProjectPointCloudToImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Matrix4f& transformation,
                              const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, const cv::Mat& image,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud) {
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  cv::Mat undistorted_image;
  cv::undistort(image, undistorted_image, camera_matrix, dist_coeffs);

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  cv::Mat zero_dist_coeffs = cv::Mat::zeros(5, 1, CV_32F);

  std::vector<cv::Point3f> object_points(1);
  std::vector<cv::Point2f> image_points(1);

  for (const auto& point : *cloud) {
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    if (transformed_point(2) < 0) continue;

    object_points[0] = cv::Point3f(transformed_point(0), transformed_point(1), transformed_point(2));
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, zero_dist_coeffs, image_points);

    int u = static_cast<int>(image_points[0].x);
    int v = static_cast<int>(image_points[0].y);

    if (u >= 0 && u < undistorted_image.cols && v >= 0 && v < undistorted_image.rows) {
      cv::Vec3b color = undistorted_image.at<cv::Vec3b>(v, u);

      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

void SaveTargetHoleCenters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& qr_centers, const Params& params) {
  if (lidar_centers->size() != 4 || qr_centers->size() != 4) {
    std::cerr << "[SaveTargetHoleCenters] The number of points in "
                 "lidar_centers or qr_centers is not 4, skip saving."
              << std::endl;
    return;
  }

  std::string save_dir = params.output_path;
  if (save_dir.back() != '/') save_dir += '/';
  std::ofstream save_file(save_dir + "circle_center_record.txt", std::ios::app);

  if (!save_file.is_open()) {
    std::cerr << "[SaveTargetHoleCenters] Cannot open file: " << save_dir + "circle_center_record.txt" << std::endl;
    return;
  }

  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  save_file << "time: " << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S") << std::endl;

  save_file << "lidar_centers:";
  for (const auto& pt : lidar_centers->points) {
    save_file << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
  }
  save_file << std::endl;
  save_file << "qr_centers:";
  for (const auto& pt : qr_centers->points) {
    save_file << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
  }
  save_file << std::endl;
  save_file.close();
  std::cout << COLOR_BOLD_GREEN << "[Record] Saved four pairs of circular hole centers to " << COLOR_BOLD_WHITE
            << save_dir << "circle_center_record.txt" << COLOR_RESET << std::endl;
}

void SaveCalibrationResults(const Params& params, const Eigen::Matrix4f& transformation,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, const cv::Mat& img_input) {
  if (colored_cloud->empty()) {
    std::cerr << COLOR_BOLD_RED << "[SaveCalibrationResults] Colored point cloud is empty!" << COLOR_RESET << std::endl;
    return;
  }
  std::string output_dir = params.output_path;
  if (output_dir.back() != '/') output_dir += '/';

  std::ofstream out_file(output_dir + "calib_result.txt");
  if (out_file.is_open()) {
    out_file << "# LIDAR-CAMERA-CALIB-FORMAT calibration format\n";
    out_file << "cam_model: Pinhole\n";
    out_file << "cam_width: " << img_input.cols << "\n";
    out_file << "cam_height: " << img_input.rows << "\n";
    out_file << "scale: 1.0\n";
    out_file << "cam_fx: " << params.fx << "\n";
    out_file << "cam_fy: " << params.fy << "\n";
    out_file << "cam_cx: " << params.cx << "\n";
    out_file << "cam_cy: " << params.cy << "\n";
    if (params.distortion_coeffs.size() >= 4) {
      out_file << "cam_d0: " << params.distortion_coeffs[0] << "\n";
      out_file << "cam_d1: " << params.distortion_coeffs[1] << "\n";
      out_file << "cam_d2: " << params.distortion_coeffs[2] << "\n";
      out_file << "cam_d3: " << params.distortion_coeffs[3] << "\n";
    }

    out_file << "\nRcl: [" << std::fixed << std::setprecision(6);
    out_file << std::setw(10) << transformation(0, 0) << ", " << std::setw(10) << transformation(0, 1) << ", "
             << std::setw(10) << transformation(0, 2) << ",\n";
    out_file << "      " << std::setw(10) << transformation(1, 0) << ", " << std::setw(10) << transformation(1, 1)
             << ", " << std::setw(10) << transformation(1, 2) << ",\n";
    out_file << "      " << std::setw(10) << transformation(2, 0) << ", " << std::setw(10) << transformation(2, 1)
             << ", " << std::setw(10) << transformation(2, 2) << "]\n";

    out_file << "Pcl: [";
    out_file << std::setw(10) << transformation(0, 3) << ", " << std::setw(10) << transformation(1, 3) << ", "
             << std::setw(10) << transformation(2, 3) << "]\n";

    out_file.close();
    std::cout << COLOR_BOLD_YELLOW << "[Result] Calibration results saved to " << COLOR_BOLD_WHITE << output_dir
              << "calib_result.txt" << COLOR_RESET << std::endl;
  } else {
    std::cerr << COLOR_BOLD_RED << "[Error] Failed to open calib_result.txt for writing!" << COLOR_RESET << std::endl;
  }

  if (pcl::io::savePCDFileASCII(output_dir + "colored_cloud.pcd", *colored_cloud) == 0) {
    std::cout << COLOR_BOLD_YELLOW << "[Result] Saved colored point cloud to: " << COLOR_BOLD_WHITE << output_dir
              << "colored_cloud.pcd" << COLOR_RESET << std::endl;
  } else {
    std::cerr << COLOR_BOLD_RED << "[Error] Failed to save colored point cloud to " << output_dir << "colored_cloud.pcd"
              << "!" << COLOR_RESET << std::endl;
  }

  cv::imwrite(output_dir + "qr_detect.png", img_input);
}

Square::Square(std::vector<pcl::PointXYZI> candidates, float width, float height) {
  candidates_ = std::move(candidates);
  target_width_ = width;
  target_height_ = height;
  target_diagonal_ = std::sqrt(std::pow(width, 2) + std::pow(height, 2));

  center_.x = center_.y = center_.z = 0;
  center_.intensity = 0;
  for (int i = 0; i < static_cast<int>(candidates_.size()); ++i) {
    center_.x += candidates_[i].x;
    center_.y += candidates_[i].y;
    center_.z += candidates_[i].z;
  }

  center_.x /= candidates_.size();
  center_.y /= candidates_.size();
  center_.z /= candidates_.size();
}

float Square::Distance(pcl::PointXYZI pt1, pcl::PointXYZI pt2) {
  return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2) + std::pow(pt1.z - pt2.z, 2));
}

pcl::PointXYZI Square::At(int i) {
  assert(0 <= i && i < 4);
  return candidates_[static_cast<size_t>(i)];
}

bool Square::IsValid(const std::string& camera_id) {
  if (candidates_.size() != 4) return false;

  for (int i = 0; i < static_cast<int>(candidates_.size()); ++i) {
    float d = Distance(center_, candidates_[static_cast<size_t>(i)]);
    if (fabs(d - target_diagonal_ / 2.f) / (target_diagonal_ / 2.f) > kGeometryTolerance * 2.0f) {
      return false;
    }
  }

  // 【修复】正确地从 std::vector 创建 PointCloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr candidates_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  candidates_cloud->points.assign(candidates_.begin(), candidates_.end());
  candidates_cloud->width = candidates_.size();
  candidates_cloud->height = 1;
  candidates_cloud->is_dense = true;

  pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_centers(new pcl::PointCloud<pcl::PointXYZI>());

  ::SortPatternCenters(candidates_cloud, sorted_centers, "camera");

  float s01 = Distance(sorted_centers->points[0], sorted_centers->points[1]);
  float s12 = Distance(sorted_centers->points[1], sorted_centers->points[2]);
  float s23 = Distance(sorted_centers->points[2], sorted_centers->points[3]);
  float s30 = Distance(sorted_centers->points[3], sorted_centers->points[0]);
  bool pattern1_ok = (fabs(s01 - target_width_) / target_width_ < kGeometryTolerance) &&
                     (fabs(s12 - target_height_) / target_height_ < kGeometryTolerance) &&
                     (fabs(s23 - target_width_) / target_width_ < kGeometryTolerance) &&
                     (fabs(s30 - target_height_) / target_height_ < kGeometryTolerance);

  bool pattern2_ok = (fabs(s01 - target_height_) / target_height_ < kGeometryTolerance) &&
                     (fabs(s12 - target_width_) / target_width_ < kGeometryTolerance) &&
                     (fabs(s23 - target_height_) / target_height_ < kGeometryTolerance) &&
                     (fabs(s30 - target_width_) / target_width_ < kGeometryTolerance);

  if (!pattern1_ok && !pattern2_ok) {
    return false;
  }

  float perimeter = s01 + s12 + s23 + s30;
  float ideal_perimeter = 2.f * (target_width_ + target_height_);
  if (fabs(perimeter - ideal_perimeter) / ideal_perimeter > kGeometryTolerance) {
    return false;
  }

  return true;
}

bool TransformLidarPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                          int direction) {
  if (!input_cloud || !output_cloud) {
    std::cerr << "[TransformLidarPoints] 输入或输出点云为空!" << std::endl;
    return false;
  }

  if (direction < 0 || direction > 3) {
    std::cerr << "[TransformLidarPoints] 无效的方向参数: " << direction
              << "，必须在 0-3 之间 (0:正前方,1:背后,2:左侧,3:右侧)" << std::endl;
    return false;
  }

  output_cloud->clear();
  output_cloud->reserve(input_cloud->size());

  for (const auto& pt : input_cloud->points) {
    pcl::PointXYZI transformed_pt = pt;  // 复制原始点云，保持intensity不变

    switch (direction) {
      case 0:  // 正前方，坐标不变
        break;
      case 1:  // 雷达在相机背后，绕Z轴旋转180度
        transformed_pt.x = -pt.x;
        transformed_pt.y = -pt.y;
        break;
      case 2:  // 雷达在相机左侧，绕Z轴旋转-90度 (顺时针)
        transformed_pt.x = pt.y;
        transformed_pt.y = -pt.x;
        break;
      case 3:  // 雷达在相机右侧，绕Z轴旋转90度 (逆时针)
        transformed_pt.x = -pt.y;
        transformed_pt.y = pt.x;
        break;
      default:
        std::cerr << "[TransformLidarPoints] 未知的方向: " << direction
                  << "，点云未被转换。" << std::endl;
        break;
    }

    output_cloud->push_back(transformed_pt);
  }

  return true;
}

Eigen::Matrix3d GetCorrectionRotationMatrix(int direction) {
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

  switch (direction) {
    case 0:  // 正前方，无旋转
      // 已经是单位矩阵
      break;
    case 1:  // 相机与雷达反向 (绕Z轴旋转180度)
      rotation_matrix << -1,  0,  0,
                          0, -1,  0,
                          0,  0,  1;
      break;
    case 2:  // 相机在雷达左侧 (绕Z轴旋转90度)
      rotation_matrix <<  0,  1,  0,
                         -1,  0,  0,
                          0,  0,  1;
      break;
    case 3:  // 相机在雷达右侧 (绕Z轴旋转-90度)
      rotation_matrix <<  0, -1,  0,
                          1,  0,  0,
                          0,  0,  1;
      break;
    default:
      std::cerr << "[GetCorrectionRotationMatrix] 无效的方向参数: " << direction
                << "，只允许 0,1,2,3。使用单位矩阵。" << std::endl;
      break;
  }

  return rotation_matrix;
}

Eigen::Matrix4d ApplyCorrectionToTransform(const Eigen::Matrix4d& original_transform, int direction) {
  // 创建修正变换矩阵
  Eigen::Matrix4d correction_transform = Eigen::Matrix4d::Identity();
  correction_transform.block<3, 3>(0, 0) = GetCorrectionRotationMatrix(direction);

  // 最终变换 = 原始变换 * 修正变换
  Eigen::Matrix4d final_transform = original_transform * correction_transform;

  return final_transform;
}

double GetCameraDirectionAngle(int direction) {
  switch (direction) {
    case 0: return   0.0;
    case 1: return 180.0;
    case 2: return -90.0;
    case 3: return  90.0;
    default:
      std::cerr << "[GetCameraDirectionAngle] 无效的 direction: " << direction << "，返回 0.0" << std::endl;
      return 0.0;
  }
}

bool TransformLidarPointsByAngle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                                 double angle_deg) {
  if (!input_cloud || !output_cloud) {
    std::cerr << "[TransformLidarPointsByAngle] 输入或输出点云为空!" << std::endl;
    return false;
  }

  const double angle_rad = angle_deg * M_PI / 180.0;
  const double cos_a = std::cos(angle_rad);
  const double sin_a = std::sin(angle_rad);

  output_cloud->clear();
  output_cloud->reserve(input_cloud->size());

  for (const auto& pt : input_cloud->points) {
    pcl::PointXYZI transformed_pt = pt;
    transformed_pt.x = static_cast<float>(cos_a * pt.x - sin_a * pt.y);
    transformed_pt.y = static_cast<float>(sin_a * pt.x + cos_a * pt.y);
    output_cloud->push_back(transformed_pt);
  }

  return true;
}

Eigen::Matrix4d ApplyCorrectionByAngle(const Eigen::Matrix4d& original_transform, double angle_deg) {
  const double angle_rad = angle_deg * M_PI / 180.0;
  const double cos_a = std::cos(angle_rad);
  const double sin_a = std::sin(angle_rad);

  Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
  correction(0, 0) =  cos_a;  correction(0, 1) = -sin_a;
  correction(1, 0) =  sin_a;  correction(1, 1) =  cos_a;

  return original_transform * correction;
}
