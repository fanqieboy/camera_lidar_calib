/**
 * @file calib_core.cpp
 * @brief 标定核心模块实现文件
 *
 * 该文件实现了LiDAR-相机外参标定的核心算法，包括单场景和多场景标定功能。
 * 使用SVD方法进行刚体变换估计，支持几何验证和数据质量检查。
 */

#include "calib_core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>

#include "common_lib.h"

// 静态常量定义
const double CalibCore::kDefaultGeometryTolerance = 0.06;

CalibCore::CalibCore(const Params& params)
    : geometry_tolerance_(kDefaultGeometryTolerance),
      expected_num_circles_(kTargetNumCircles),
      verbose_(false),
      total_calibrations_(0),
      successful_calibrations_(0),
      total_processing_time_(0.0) {
  // 从参数中提取标定板尺寸信息
  target_width_ = params.delta_width_circles;
  target_height_ = params.delta_height_circles;
  camera_id_ = params.camera_id;  // 【新增】从参数中获取相机ID
  camera_direction_ = params.camera_direction;  // 【新增】从参数中获取相机方向

  if (verbose_) {
    std::cout << "[CalibCore] Initialized with target dimensions: " << target_width_ << " x " << target_height_
              << ", camera_direction: " << camera_direction_ << std::endl;
  }
}

CalibCore::~CalibCore() {
  if (verbose_ && total_calibrations_ > 0) {
    // 输出统计信息
    std::cout << "[CalibCore] Statistics: " << successful_calibrations_ << "/" << total_calibrations_
              << " successful calibrations, "
              << "average time: " << (total_processing_time_ / total_calibrations_) << "ms" << std::endl;
  }
}

RigidTransformResult CalibCore::PerformSingleSceneCalibration(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_centers) {
  auto start_time = std::chrono::high_resolution_clock::now();  // 记录开始时间
  total_calibrations_++;  // 增加总标定次数

  RigidTransformResult result;  // 初始化结果

  // 验证输入数据
  if (!ValidateInputData(lidar_centers, camera_centers)) {
    result.error_message = last_error_;
    return result;
  }

  try {
    // 对点进行排序以确保对应关系正确
    pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_lidar_centers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_camera_centers(new pcl::PointCloud<pcl::PointXYZI>());

    // 【修复】调用 static 函数时，传入成员变量 camera_id_
    ::SortPatternCenters(lidar_centers, sorted_lidar_centers, "lidar");
    ::SortPatternCenters(camera_centers, sorted_camera_centers,  "camera");

    // 使用PCL的SVD方法估计刚体变换
    Eigen::Matrix4f transformation_f;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI> svd;
    svd.estimateRigidTransformation(*sorted_lidar_centers, *sorted_camera_centers, transformation_f);

    // 转换为双精度
    result.transformation = transformation_f.cast<double>();
    result.rotation = result.transformation.block<3, 3>(0, 0);
    result.translation = result.transformation.block<3, 1>(0, 3);

    // 计算RMSE
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_lidar(new pcl::PointCloud<pcl::PointXYZI>());
    TransformPointCloud(sorted_lidar_centers, transformed_lidar, result.transformation);
    result.rmse = ComputeRMSE(sorted_camera_centers, transformed_lidar);

    if (result.rmse >= 0) {
      result.success = true;
      result.num_points = sorted_lidar_centers->size();
      successful_calibrations_++;

      if (verbose_) {
        std::cout << "[CalibCore] Single-scene calibration successful. RMSE: " << std::fixed << std::setprecision(4)
                  << result.rmse << " m" << std::endl;
      }
    } else {
      result.error_message = "RMSE calculation failed";
      last_error_ = result.error_message;
    }
  } catch (const std::exception& e) {
    result.error_message = "SVD estimation failed: " + std::string(e.what());
    last_error_ = result.error_message;
  }

  // 记录处理时间
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  total_processing_time_ += duration.count();

  return result;
}

RigidTransformResult CalibCore::PerformMultiSceneCalibration(const std::vector<CalibrationBlock>& calibration_blocks,
                                                             const std::vector<double>* weights) {
  auto start_time = std::chrono::high_resolution_clock::now();
  total_calibrations_++;

  RigidTransformResult result;

  if (calibration_blocks.size() < 3) {
    result.error_message = "Need at least 3 calibration blocks for multi-scene calibration";
    last_error_ = result.error_message;
    return result;
  }

  try {
    // 将所有数据块合并为点对列表
    std::vector<Eigen::Vector3d> lidar_points, camera_points;

    for (const auto& block : calibration_blocks) {
      if (!block.is_valid || block.lidar_centers->size() != kTargetNumCircles ||
          block.camera_centers->size() != kTargetNumCircles) {
        if (verbose_) {
          std::cout << "[CalibCore] Skipping invalid calibration block" << std::endl;
        }
        continue;
      }

      // 对每个数据块的点进行排序
      pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_lidar(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_camera(new pcl::PointCloud<pcl::PointXYZI>());

      // 【修复】调用 static 函数时，传入成员变量 camera_id_
      ::SortPatternCenters(block.lidar_centers, sorted_lidar, "lidar");
      ::SortPatternCenters(block.camera_centers, sorted_camera, "camera");

      // 添加到点对列表
      for (size_t i = 0; i < kTargetNumCircles; ++i) {
        const auto& lpt = sorted_lidar->points[i];
        const auto& cpt = sorted_camera->points[i];

        lidar_points.emplace_back(lpt.x, lpt.y, lpt.z);
        camera_points.emplace_back(cpt.x, cpt.y, cpt.z);
      }
    }

    if (lidar_points.size() < 12) {
      // 至少3个场景 x 4个点
      result.error_message = "Insufficient valid calibration data points";
      last_error_ = result.error_message;
      return result;
    }

    // 输出详细的调试信息（与原始ROS版本匹配）
    if (verbose_) {
      std::cout << "Using " << calibration_blocks.size() << " datasets for multi-scene calibration." << std::endl;
      std::cout << "Total LiDAR centers: " << lidar_points.size() << std::endl;
      for (size_t i = 0; i < lidar_points.size(); ++i) {
        std::cout << "L[" << i << "]: (" << lidar_points[i](0) << ", " << lidar_points[i](1) << ", "
                  << lidar_points[i](2) << ")" << std::endl;
      }
      std::cout << "Total QR centers: " << camera_points.size() << std::endl;
      for (size_t i = 0; i < camera_points.size(); ++i) {
        std::cout << "C[" << i << "]: (" << camera_points[i](0) << ", " << camera_points[i](1) << ", "
                  << camera_points[i](2) << ")" << std::endl;
      }
    }

    // 执行加权刚体变换求解
    result = SolveWeightedRigidTransform(lidar_points, camera_points, weights);

    if (result.success) {
      result.num_points = lidar_points.size();
      successful_calibrations_++;

      if (verbose_) {
        std::cout << "[CalibCore] Multi-scene calibration successful using " << calibration_blocks.size() << " blocks, "
                  << lidar_points.size() << " point pairs. RMSE: " << std::fixed << std::setprecision(4) << result.rmse
                  << " m" << std::endl;
      }
    }
  } catch (const std::exception& e) {
    result.error_message = "Multi-scene calibration failed: " + std::string(e.what());
    last_error_ = result.error_message;
  }

  // 记录处理时间
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  total_processing_time_ += duration.count();

  return result;
}

RigidTransformResult CalibCore::PerformMultiSceneCalibration(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& lidar_centers_list,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& camera_centers_list, const std::vector<double>* weights) {
  if (lidar_centers_list.size() != camera_centers_list.size()) {
    RigidTransformResult result;
    result.error_message = "Mismatched number of lidar and camera center lists";
    last_error_ = result.error_message;
    return result;
  }

  // 转换为CalibrationBlock格式
  std::vector<CalibrationBlock> blocks;
  for (size_t i = 0; i < lidar_centers_list.size(); ++i) {
    CalibrationBlock block;
    block.lidar_centers = lidar_centers_list[i];
    block.camera_centers = camera_centers_list[i];
    block.is_valid =
        (block.lidar_centers->size() == kTargetNumCircles && block.camera_centers->size() == kTargetNumCircles);

    // 生成时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    block.timestamp = ss.str();

    blocks.push_back(block);
  }

  return PerformMultiSceneCalibration(blocks, weights);
}

double CalibCore::ComputeRMSE(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2) {
  if (!cloud1 || !cloud2 || cloud1->size() != cloud2->size()) {
    return -1.0;
  }

  if (cloud1->empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (size_t i = 0; i < cloud1->size(); ++i) {
    double dx = cloud1->points[i].x - cloud2->points[i].x;
    double dy = cloud1->points[i].y - cloud2->points[i].y;
    double dz = cloud1->points[i].z - cloud2->points[i].z;
    sum += dx * dx + dy * dy + dz * dz;
  }

  return std::sqrt(sum / cloud1->size());
}

void CalibCore::TransformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                                    const Eigen::Matrix4d& transformation) {
  if (!input_cloud) {
    return;
  }

  output_cloud->clear();
  output_cloud->reserve(input_cloud->size());

  for (const auto& pt : input_cloud->points) {
    Eigen::Vector4d pt_homogeneous(pt.x, pt.y, pt.z, 1.0);
    Eigen::Vector4d transformed_pt = transformation * pt_homogeneous;

    pcl::PointXYZI output_pt;
    output_pt.x = transformed_pt(0);
    output_pt.y = transformed_pt(1);
    output_pt.z = transformed_pt(2);
    output_pt.intensity = pt.intensity;
    output_cloud->push_back(output_pt);
  }
}

bool CalibCore::SaveCalibrationData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_centers,
                                    const std::string& output_path, const std::string& filename) {
  if (!lidar_centers || !camera_centers || lidar_centers->size() != kTargetNumCircles ||
      camera_centers->size() != kTargetNumCircles) {
    return false;
  }

  std::string save_dir = output_path;
  if (save_dir.back() != '/') save_dir += '/';
  std::string filepath = save_dir + filename;

  std::ofstream save_file(filepath, std::ios::app);
  if (!save_file.is_open()) {
    return false;
  }

  // 获取当前时间
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  save_file << "time: " << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << std::endl;

  // 保存LiDAR圆心
  save_file << "lidar_centers:";
  for (const auto& pt : lidar_centers->points) {
    save_file << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
  }
  save_file << std::endl;

  // 保存相机圆心
  save_file << "qr_centers:";
  for (const auto& pt : camera_centers->points) {
    save_file << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
  }
  save_file << std::endl;

  save_file.close();
  return true;
}

std::vector<CalibrationBlock> CalibCore::LoadCalibrationDataFromFile(const std::string& record_file,
                                                                     const std::vector<int>& selected_indices) {
  std::vector<CalibrationBlock> blocks;

  std::ifstream file(record_file);
  if (!file.is_open()) {
    return blocks;
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  file.close();

  // 解析所有数据块（按三行一组：time + lidar_centers + qr_centers）
  std::vector<CalibrationBlock> all_blocks;
  for (size_t i = 0; i + 2 < lines.size(); ++i) {
    if (lines[i].rfind("time:", 0) == 0 && lines[i + 1].find("lidar_centers:") != std::string::npos &&
        lines[i + 2].find("qr_centers:") != std::string::npos) {
      CalibrationBlock block;
      block.timestamp = lines[i].substr(6);  // 去掉"time: "

      std::vector<Eigen::Vector3d> lidar_pts, camera_pts;
      if (ParseCentersLine(lines[i + 1], lidar_pts) && ParseCentersLine(lines[i + 2], camera_pts) &&
          lidar_pts.size() == kTargetNumCircles && camera_pts.size() == kTargetNumCircles) {
        // 转换为PCL点云格式
        for (const auto& pt : lidar_pts) {
          pcl::PointXYZI pcl_pt;
          pcl_pt.x = pt(0);
          pcl_pt.y = pt(1);
          pcl_pt.z = pt(2);
          pcl_pt.intensity = 0.0;
          block.lidar_centers->push_back(pcl_pt);
        }

        for (const auto& pt : camera_pts) {
          pcl::PointXYZI pcl_pt;
          pcl_pt.x = pt(0);
          pcl_pt.y = pt(1);
          pcl_pt.z = pt(2);
          pcl_pt.intensity = 0.0;
          block.camera_centers->push_back(pcl_pt);
        }

        block.is_valid = true;
        all_blocks.push_back(block);
      }

      i += 2;  // 跳过这个数据块
    }
  }

  // 根据选择的索引返回数据块
  if (selected_indices.empty()) {
    return all_blocks;  // 返回所有数据块
  } else {
    for (int idx : selected_indices) {
      if (idx >= 0 && idx < static_cast<int>(all_blocks.size())) {
        blocks.push_back(all_blocks[idx]);
      }
    }
    return blocks;
  }
}

void CalibCore::UpdateParameters(const Params& params) {
  target_width_ = params.delta_width_circles;
  target_height_ = params.delta_height_circles;

  if (verbose_) {
    std::cout << "[CalibCore] Parameters updated. Target dimensions: " << target_width_ << " x " << target_height_
              << std::endl;
  }
}

void CalibCore::SetVerbose(bool verbose) { verbose_ = verbose; }

std::string CalibCore::GetLastError() const { return last_error_; }

std::string CalibCore::GetCalibrationStats() const {
  std::stringstream ss;
  ss << "Calibration Statistics:\n";
  ss << "  Total calibrations: " << total_calibrations_ << "\n";
  ss << "  Successful calibrations: " << successful_calibrations_ << "\n";
  ss << "  Success rate: " << (total_calibrations_ > 0 ? (100.0 * successful_calibrations_ / total_calibrations_) : 0.0)
     << "%\n";
  ss << "  Average processing time: "
     << (total_calibrations_ > 0 ? (total_processing_time_ / total_calibrations_) : 0.0) << " ms";
  return ss.str();
}

// 私有方法实现

RigidTransformResult CalibCore::SolveWeightedRigidTransform(const std::vector<Eigen::Vector3d>& lidar_points,
                                                            const std::vector<Eigen::Vector3d>& camera_points,
                                                            const std::vector<double>* weights) const {
  RigidTransformResult result;

  const size_t N = lidar_points.size();
  if (N < 3 || camera_points.size() != N) {
    result.error_message = "Insufficient or mismatched point pairs";
    return result;
  }

  try {
    // 权重处理
    std::vector<double> w(N, 1.0);
    if (weights && weights->size() == N) {
      w = *weights;
    }

    double wsum = 0.0;
    for (double wi : w) wsum += wi;
    if (wsum <= 0) {
      result.error_message = "Invalid weights";
      return result;
    }

    // 计算加权质心
    Eigen::Vector3d muL = Eigen::Vector3d::Zero();
    Eigen::Vector3d muC = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      muL += w[i] * lidar_points[i];
      muC += w[i] * camera_points[i];
    }
    muL /= wsum;
    muC /= wsum;

    // 计算协方差矩阵
    Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector3d l = lidar_points[i] - muL;
      Eigen::Vector3d c = camera_points[i] - muC;
      Sigma += w[i] * (l * c.transpose());
    }

    // SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();

    // 确保旋转矩阵的行列式为正
    if (R.determinant() < 0) {
      Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
      D(2, 2) = -1;
      R = V * D * U.transpose();
    }

    // 计算平移向量
    Eigen::Vector3d t = muC - R * muL;

    // 计算RMSE
    double rss = 0.0;
    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector3d residual = (R * lidar_points[i] + t) - camera_points[i];
      rss += w[i] * residual.squaredNorm();
    }

    // 设置结果
    result.rotation = R;
    result.translation = t;
    result.transformation = Eigen::Matrix4d::Identity();
    result.transformation.block<3, 3>(0, 0) = R;
    result.transformation.block<3, 1>(0, 3) = t;
    result.rmse = std::sqrt(rss / wsum);
    result.success = true;
    result.num_points = N;
  } catch (const std::exception& e) {
    result.error_message = "SVD computation failed: " + std::string(e.what());
  }

  return result;
}

bool CalibCore::ParseCentersLine(const std::string& line, std::vector<Eigen::Vector3d>& centers) {
  centers.clear();

  // 正则表达式匹配形如 {x,y,z} 的模式
  std::regex brace_re("\\{([^\\}]*)\\}");
  auto begin = std::sregex_iterator(line.begin(), line.end(), brace_re);
  auto end = std::sregex_iterator();

  for (auto it = begin; it != end; ++it) {
    std::string xyz = (*it)[1];  // 获取大括号内的内容

    // 去除空格
    xyz.erase(std::remove_if(xyz.begin(), xyz.end(), ::isspace), xyz.end());

    // 用逗号分割
    std::vector<double> vals;
    std::stringstream ss(xyz);
    std::string token;
    while (std::getline(ss, token, ',')) {
      try {
        vals.push_back(std::stod(token));
      } catch (...) {
        return false;
      }
    }

    if (vals.size() != 3) {
      return false;
    }

    centers.emplace_back(vals[0], vals[1], vals[2]);
  }

  return !centers.empty();
}

bool CalibCore::ValidateInputData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_centers) const {
  if (!lidar_centers || !camera_centers) {
    last_error_ = "Null input point clouds";
    return false;
  }

  if (lidar_centers->size() != expected_num_circles_ || camera_centers->size() != expected_num_circles_) {
    last_error_ =
        "Point cloud sizes do not match expected number of circles (" + std::to_string(expected_num_circles_) + ")";
    return false;
  }

  if (lidar_centers->size() != camera_centers->size()) {
    last_error_ = "LiDAR and camera point cloud sizes do not match";
    return false;
  }

  return true;
}