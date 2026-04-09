/**
 * @file lidar_detector.cpp
 * @brief LiDAR点云检测模块实现文件
 *
 * 实现了独立的LiDAR点云检测算法，支持从点云中检测圆形标定板。
 */

#include "lidar_detector.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "config_manager.h"
#include "common_lib.h"
#include "qr_detector.h"

LidarDetector::LidarDetector(const Params& params)
    : verbose_(false), total_detections_(0), successful_detections_(0), total_processing_time_(0.0), camera_id_("") {
  InitializeParameters(params);

  if (verbose_) {
    std::cout << "[LidarDetector] 初始化完成" << std::endl;
    std::cout << "  滤波范围 - X: [" << x_min_ << ", " << x_max_ << "]" << std::endl;
    std::cout << "  滤波范围 - Y: [" << y_min_ << ", " << y_max_ << "]" << std::endl;
    std::cout << "  滤波范围 - Z: [" << z_min_ << ", " << z_max_ << "]" << std::endl;
    std::cout << "  圆形半径: " << circle_radius_ << "m" << std::endl;
    std::cout << "  相机方向: " << camera_direction_ << std::endl;
  }
}

LidarDetector::~LidarDetector() = default;

void LidarDetector::InitializeParameters(const Params& params) {
  // 滤波参数
  x_min_ = params.x_min;
  x_max_ = params.x_max;
  y_min_ = params.y_min;
  y_max_ = params.y_max;
  z_min_ = params.z_min;
  z_max_ = params.z_max;
  voxel_size_ = 0.005;
  intensity_min_ = 0.0;
  intensity_max_ = 130.0;

  // 检测参数
  circle_radius_ = params.circle_radius;

  // 平面分割参数
  plane_distance_threshold_ = 0.01;
  plane_max_iterations_ = 1000;

  // 边界检测参数
  boundary_radius_search_ = 0.03;
  boundary_angle_threshold_ = M_PI / 4;

  // 聚类参数
  cluster_tolerance_ = 0.02;
  min_cluster_size_ = 50;
  // 初始设置为1000
  max_cluster_size_ = 20000;

  // 圆形拟合参数
  circle_distance_threshold_ = 0.01;
  circle_max_iterations_ = 1000;
  fitting_error_threshold_ = 0.025;

  // 相机方向参数
  camera_direction_ = params.camera_direction;
  lidar_yaw_offset_deg_ = params.lidar_yaw_offset_deg;
}

LidarProcessingResult LidarDetector::DetectCircles(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  auto start_time = std::chrono::high_resolution_clock::now();

  LidarProcessingResult result;
  total_detections_++;

  if (!cloud || cloud->empty()) {
    result.error_message = "输入点云为空";
    last_error_ = result.error_message;
    return result;
  }

  try {
    // 根据 camera_direction + lidar_yaw_offset_deg 旋转点云到与相机同向的坐标系
    pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (lidar_yaw_offset_deg_ != 0.0) {
      // 有附加偏差：合并两个角度后统一用连续角度旋转
      double total_yaw = GetCameraDirectionAngle(camera_direction_) + lidar_yaw_offset_deg_;
      if (!TransformLidarPointsByAngle(cloud, rotated_cloud, total_yaw)) {
        result.error_message = "点云旋转失败 (angle=" + std::to_string(total_yaw) + "deg)";
        return result;
      }
    } else {
      // 无附加偏差：沿用原有离散旋转逻辑
      if (!TransformLidarPoints(cloud, rotated_cloud, camera_direction_)) {
        result.error_message = "点云旋转失败";
        return result;
      }
    }
    // 保存旋转后点云，供失败诊断时输出
    result.rotated_cloud = rotated_cloud;

    // 1. 点云滤波
    if (!FilterPointCloud(rotated_cloud, result.filtered_cloud, result.noise_cloud)) {
      result.error_message = "点云滤波失败";
      return result;
    }

    if (result.filtered_cloud->empty()) {
      result.error_message = "滤波后点云为空";
      return result;
    }

    // 2. 平面分割
    if (!SegmentPlane(result.filtered_cloud, result.plane_cloud, result.plane_coefficients)) {
      result.error_message = "平面分割失败";
      return result;
    }

    if (result.plane_cloud->empty()) {
      result.error_message = "平面点云为空";
      return result;
    }

    // 3. 平面对齐
    Eigen::Matrix3d rotation_matrix;
    double average_z;
    if (!AlignPlaneToZ0(result.plane_cloud, result.aligned_cloud, result.plane_coefficients, rotation_matrix,
                        average_z)) {
      result.error_message = "平面对齐失败";
      return result;
    }

    // 4. 边界检测
    if (!DetectBoundaryPoints(result.aligned_cloud, result.edge_cloud)) {
      result.error_message = "边界检测失败";
      return result;
    }

    if (result.edge_cloud->empty()) {
      result.error_message = "边界点云为空";
      return result;
    }

    // 5. 聚类和圆形拟合
    if (!ClusterAndFitCircles(result.edge_cloud, result.center_z0_cloud, result.circle_coefficients)) {
      result.error_message = "圆形拟合失败";
      return result;
    }

    // 6. 转换回原坐标系
    TransformCentersBack(result.center_z0_cloud, result.circle_centers, rotation_matrix, average_z);
    // --- START: 新增排序逻辑 ---
    // 检查检测到的圆心数量是否为目标数量（通常是4），以确保可以进行有效排序
    if (result.circle_centers->size() == kTargetNumCircles) {
      // 创建一个临时的智能指针来存放排序后的点
      pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_centers(new pcl::PointCloud<pcl::PointXYZI>());

      // 调用公共库中的排序函数，确保与QR检测器使用相同的排序规则。
      // 最后一个参数 "lidar" 至关重要，它确保在LiDAR坐标系下进行排序。
      // 第二个参数 "default_lidar" 是一个占位符，因为LiDAR没有相机ID，具体值不影响排序逻辑。
      ::SortPatternCenters(result.circle_centers, sorted_centers, "default_lidar");

      // 用排序后的点云替换原来的点云
      result.circle_centers = sorted_centers;

      if (verbose_) {
        std::cout << "[LidarDetector] 圆心已按 \"lidar\" 坐标系规则排序" << std::endl;
      }
    }
    // --- END: 新增排序逻辑 ---
    result.circles_detected = result.circle_centers->size();
    result.success = true;
    successful_detections_++;

    auto end_time = std::chrono::high_resolution_clock::now();
    result.processing_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    total_processing_time_ += result.processing_time_ms;

    if (verbose_) {
      std::cout << "[LidarDetector] 检测成功: " << result.circles_detected
                << " 个圆心, 处理时间: " << result.processing_time_ms << "ms" << std::endl;
    }
  } catch (const std::exception& e) {
    result.error_message = "处理错误: " + std::string(e.what());
    last_error_ = result.error_message;
  }

  return result;
}

bool LidarDetector::DetectLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud) {
  LidarProcessingResult result = DetectCircles(cloud);

  if (result.success && !result.circle_centers->empty()) {
    *center_cloud = *result.circle_centers;
    return true;
  }

  last_error_ = result.error_message;
  return false;
}

bool LidarDetector::FilterPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr noise_cloud) {
  try {
    // 强度滤波
    pcl::PassThrough<pcl::PointXYZI> pass_intensity;
    pass_intensity.setInputCloud(cloud);
    pass_intensity.setFilterFieldName("intensity");
    pass_intensity.setFilterLimits(intensity_min_, intensity_max_);
    pass_intensity.filter(*filtered_cloud);

    // 噪声点（高强度）
    pass_intensity.setFilterLimits(intensity_max_, std::numeric_limits<float>::max());
    pass_intensity.filter(*noise_cloud);

    if (verbose_) {
      std::cout << "[LidarDetector] 噪声点数量 (强度 > " << intensity_max_ << "): " << noise_cloud->size() << std::endl;
    }

    // X 方向滤波
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(filtered_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_);
    pass_x.filter(*filtered_cloud);

    // Y 方向滤波
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(filtered_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_);
    pass_y.filter(*filtered_cloud);

    // Z 方向滤波
    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pass_z.setInputCloud(filtered_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*filtered_cloud);

    if (verbose_) {
      std::cout << "[LidarDetector] 滤波后点云大小: " << filtered_cloud->size() << std::endl;
    }

    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*filtered_cloud);

    if (verbose_) {
      std::cout << "[LidarDetector] 体素滤波后大小: " << filtered_cloud->size() << std::endl;
    }

    return true;
  } catch (const std::exception& e) {
    last_error_ = "滤波错误: " + std::string(e.what());
    return false;
  }
}

bool LidarDetector::SegmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud,
                                 pcl::ModelCoefficients::Ptr coefficients) {
  try {
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> plane_segmentation;
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(plane_distance_threshold_);
    plane_segmentation.setMaxIterations(plane_max_iterations_);
    plane_segmentation.setInputCloud(cloud);
    plane_segmentation.segment(*plane_inliers, *coefficients);

    if (plane_inliers->indices.empty()) {
      last_error_ = "未找到平面";
      return false;
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(plane_inliers);
    extract.filter(*plane_cloud);

    if (verbose_) {
      std::cout << "[LidarDetector] 平面点云大小: " << plane_cloud->size() << std::endl;
      std::cout << "[LidarDetector] 平面方程: " << coefficients->values[0] << "x + " << coefficients->values[1]
                << "y + " << coefficients->values[2] << "z + " << coefficients->values[3] << " = 0" << std::endl;
    }

    return true;
  } catch (const std::exception& e) {
    last_error_ = "平面分割错误: " + std::string(e.what());
    return false;
  }
}

bool LidarDetector::AlignPlaneToZ0(pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud,
                                   pcl::ModelCoefficients::Ptr coefficients, Eigen::Matrix3d& rotation_matrix,
                                   double& average_z) {
  try {
    // 计算旋转矩阵将平面法向量对齐到z轴
    Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    normal.normalize();
    Eigen::Vector3d z_axis(0, 0, 1);

    Eigen::Vector3d axis = normal.cross(z_axis);
    double angle = acos(normal.dot(z_axis));

    if (axis.norm() < 1e-6) {
      // 法向量已经是z轴方向
      rotation_matrix = Eigen::Matrix3d::Identity();
    } else {
      axis.normalize();
      Eigen::AngleAxisd rotation(angle, axis);
      rotation_matrix = rotation.toRotationMatrix();
    }

    // 对齐点云并计算平均z值
    aligned_cloud->clear();
    aligned_cloud->reserve(plane_cloud->size());

    average_z = 0.0;
    int count = 0;

    for (const auto& pt : *plane_cloud) {
      Eigen::Vector3d point(pt.x, pt.y, pt.z);
      Eigen::Vector3d aligned_point = rotation_matrix * point;

      pcl::PointXYZI aligned_pt;
      aligned_pt.x = aligned_point.x();
      aligned_pt.y = aligned_point.y();
      aligned_pt.z = 0.0;  // 投影到z=0平面
      aligned_pt.intensity = pt.intensity;
      aligned_cloud->push_back(aligned_pt);

      average_z += aligned_point.z();
      count++;
    }

    average_z /= count;

    if (verbose_) {
      std::cout << "[LidarDetector] 平面对齐完成，平均z值: " << average_z << std::endl;
    }

    return true;
  } catch (const std::exception& e) {
    last_error_ = "平面对齐错误: " + std::string(e.what());
    return false;
  }
}

bool LidarDetector::DetectBoundaryPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud) {
  try {
    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setRadiusSearch(boundary_radius_search_);
    normal_estimator.compute(*normals);

    // 检测边界点
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary> boundary_estimator;
    boundary_estimator.setInputCloud(cloud);
    boundary_estimator.setInputNormals(normals);
    boundary_estimator.setRadiusSearch(boundary_radius_search_);
    boundary_estimator.setAngleThreshold(boundary_angle_threshold_);
    boundary_estimator.compute(boundaries);

    edge_cloud->clear();
    edge_cloud->reserve(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
      if (boundaries.points[i].boundary_point > 0) {
        edge_cloud->push_back(cloud->points[i]);
      }
    }

    if (verbose_) {
      std::cout << "[LidarDetector] 提取边界点: " << edge_cloud->size() << std::endl;
    }

    return !edge_cloud->empty();
  } catch (const std::exception& e) {
    last_error_ = "边界检测错误: " + std::string(e.what());
    return false;
  }
}

bool LidarDetector::ClusterAndFitCircles(pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr center_z0_cloud,
                                         std::vector<pcl::ModelCoefficients>& circle_coefficients) {
  try {
    // 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(edge_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_); // 设置聚类公差（点与点之间的最大距离）
    ec.setMinClusterSize(min_cluster_size_); // 设置最小点数
    ec.setMaxClusterSize(max_cluster_size_); // 设置最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(edge_cloud);
    ec.extract(cluster_indices);

    if (verbose_) {
      std::cout << "[LidarDetector] 边界聚类数量: " << cluster_indices.size() << std::endl;
    }

    center_z0_cloud->clear();
    circle_coefficients.clear();

    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      // 提取聚类点云
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for (const auto& idx : cluster_indices[i].indices) {
        cluster->push_back(edge_cloud->points[idx]);
      }

      // 圆形拟合
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CIRCLE2D);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(circle_distance_threshold_);
      seg.setMaxIterations(circle_max_iterations_);
      seg.setInputCloud(cluster);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) continue;

      // 验证拟合质量
      double error = 0.0;
      for (const auto& idx : inliers->indices) {
        double dx = cluster->points[idx].x - coefficients->values[0];
        double dy = cluster->points[idx].y - coefficients->values[1];
        double distance = sqrt(dx * dx + dy * dy) - circle_radius_;
        error += std::abs(distance);
      }
      error /= inliers->indices.size();

      if (error < fitting_error_threshold_) {
        pcl::PointXYZI center_point;
        center_point.x = coefficients->values[0];
        center_point.y = coefficients->values[1];
        center_point.z = 0.0;
        center_point.intensity = 0.0;
        center_z0_cloud->push_back(center_point);

        circle_coefficients.push_back(*coefficients);

        if (verbose_) {
          std::cout << "[LidarDetector] 圆形 " << i << ": 中心(" << coefficients->values[0] << ", "
                    << coefficients->values[1] << "), 半径: " << coefficients->values[2] << ", 误差: " << error
                    << std::endl;
        }
      }
    }

    return !center_z0_cloud->empty();
  } catch (const std::exception& e) {
    last_error_ = "聚类和圆拟合错误: " + std::string(e.what());
    return false;
  }
}

void LidarDetector::TransformCentersBack(pcl::PointCloud<pcl::PointXYZI>::Ptr center_z0_cloud,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud,
                                         const Eigen::Matrix3d& rotation_matrix, double average_z) {
  center_cloud->clear();
  center_cloud->reserve(center_z0_cloud->size());

  Eigen::Matrix3d R_inv = rotation_matrix.inverse();

  for (const auto& pt : *center_z0_cloud) {
    Eigen::Vector3d aligned_point(pt.x, pt.y, pt.z + average_z);
    Eigen::Vector3d original_point = R_inv * aligned_point;

    pcl::PointXYZI center_point_origin;
    center_point_origin.x = original_point.x();
    center_point_origin.y = original_point.y();
    center_point_origin.z = original_point.z();
    center_point_origin.intensity = 0.0;
    center_cloud->push_back(center_point_origin);
  }
}

// 参数更新和配置方法实现
void LidarDetector::UpdateParameters(const Params& params) { InitializeParameters(params); }

void LidarDetector::SetVerbose(bool verbose) { verbose_ = verbose; }

void LidarDetector::SetFilterParameters(double voxel_size, double intensity_min, double intensity_max) {
  voxel_size_ = voxel_size;
  intensity_min_ = intensity_min;
  intensity_max_ = intensity_max;
}

void LidarDetector::SetPlaneSegmentationParameters(double distance_threshold, int max_iterations) {
  plane_distance_threshold_ = distance_threshold;
  plane_max_iterations_ = max_iterations;
}

void LidarDetector::SetBoundaryDetectionParameters(double radius_search, double angle_threshold) {
  boundary_radius_search_ = radius_search;
  boundary_angle_threshold_ = angle_threshold;
}

void LidarDetector::SetClusteringParameters(double cluster_tolerance, int min_cluster_size, int max_cluster_size) {
  cluster_tolerance_ = cluster_tolerance;
  min_cluster_size_ = min_cluster_size;
  max_cluster_size_ = max_cluster_size;
}

void LidarDetector::SetCircleFittingParameters(double distance_threshold, int max_iterations,
                                               double fitting_error_threshold) {
  circle_distance_threshold_ = distance_threshold;
  circle_max_iterations_ = max_iterations;
  fitting_error_threshold_ = fitting_error_threshold;
}

bool LidarDetector::SaveIntermediateResults(const LidarProcessingResult& result, const std::string& output_directory,
                                            const std::string& prefix) const {
  try {
    // 创建输出目录
    std::filesystem::create_directories(output_directory);

    // 保存各个阶段的点云
    std::string base_path = output_directory + "/" + prefix;

    if (result.filtered_cloud && !result.filtered_cloud->empty()) {
      pcl::io::savePCDFileASCII(base_path + "filtered.pcd", *result.filtered_cloud);
    }

    if (result.plane_cloud && !result.plane_cloud->empty()) {
      pcl::io::savePCDFileASCII(base_path + "plane.pcd", *result.plane_cloud);
    }

    if (result.aligned_cloud && !result.aligned_cloud->empty()) {
      pcl::io::savePCDFileASCII(base_path + "aligned.pcd", *result.aligned_cloud);
    }

    if (result.edge_cloud && !result.edge_cloud->empty()) {
      pcl::io::savePCDFileASCII(base_path + "edge.pcd", *result.edge_cloud);
    }

    if (result.center_z0_cloud && !result.center_z0_cloud->empty()) {
      pcl::io::savePCDFileASCII(base_path + "centers_z0.pcd", *result.center_z0_cloud);
    }

    if (result.circle_centers && !result.circle_centers->empty()) {
      pcl::io::savePCDFileASCII(base_path + "centers.pcd", *result.circle_centers);
    }

    if (result.noise_cloud && !result.noise_cloud->empty()) {
      pcl::io::savePCDFileASCII(base_path + "noise.pcd", *result.noise_cloud);
    }

    return true;
  } catch (const std::exception& e) {
    return false;
  }
}

std::string LidarDetector::GetLastError() const { return last_error_; }

std::string LidarDetector::GetProcessingStats() const {
  const int success_rate = total_detections_ > 0 ? (successful_detections_ * 100 / total_detections_) : 0;
  const double avg_time = total_detections_ > 0 ? (total_processing_time_ / total_detections_) : 0.0;

  return "处理统计 - 总计: " + std::to_string(total_detections_) + ", 成功: " + std::to_string(successful_detections_) +
         " (" + std::to_string(success_rate) + "%)" + ", 平均处理时间: " + std::to_string(avg_time) + "ms";
}