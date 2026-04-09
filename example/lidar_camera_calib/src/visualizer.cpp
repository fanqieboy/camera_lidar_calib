/**
 * @file visualizer.cpp
 * @brief 可视化模块实现 - 移除ROS依赖的独立可视化系统
 *
 * 实现独立的3D点云和图像可视化功能，替代ROS RViz
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE' file,
 * which is included as part of this source code package.
 */

#include "visualizer.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <filesystem>
#include <pcl/io/pcd_io.h>

#include "common_lib.h"
#include "lidar_detector.h"
#include "qr_detector.h"
#include "calib_core.h"

// 静态默认颜色定义
const std::vector<std::vector<double>> Visualizer::kDefaultColors = {
    {1.0, 0.0, 0.0},  // 红色
    {0.0, 1.0, 0.0},  // 绿色
    {0.0, 0.0, 1.0},  // 蓝色
    {1.0, 1.0, 0.0},  // 黄色
    {1.0, 0.0, 1.0},  // 品红色
    {0.0, 1.0, 1.0},  // 青色
    {1.0, 0.5, 0.0},  // 橙色
    {0.5, 0.0, 1.0},  // 紫色
    {0.0, 0.5, 1.0},  // 天蓝色
    {1.0, 0.0, 0.5}   // 粉红色
};

Visualizer::Visualizer(const Params& params)
    : is_initialized_(false), verbose_(false), scene_counter_(0), file_counter_(0) {
  // 从参数中提取相机参数
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = params.fx;
  camera_matrix_.at<double>(1, 1) = params.fy;
  camera_matrix_.at<double>(0, 2) = params.cx;
  camera_matrix_.at<double>(1, 2) = params.cy;

  distortion_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
  // 添加新的逻辑:
  if (!params.distortion_coeffs.empty()) {
    // 确保 distortion_coeffs_ 矩阵有足够空间
    distortion_coeffs_ = cv::Mat(params.distortion_coeffs, true).clone();
  }

  InitializeOutputDirectory();
}

Visualizer::~Visualizer() {
  try {
    // 清理OpenCV窗口
    cv::destroyAllWindows();
    cv::waitKey(1);  // 确保窗口清理完成

    is_initialized_ = false;

  } catch (const std::exception& e) {
    // 析构函数中不能抛出异常，只记录错误
    if (verbose_) {
      std::cerr << "[Visualizer] Error in destructor: " << e.what() << std::endl;
    }
  } catch (...) {
    // 捕获所有其他异常
    if (verbose_) {
      std::cerr << "[Visualizer] Unknown error in destructor" << std::endl;
    }
  }
}

void Visualizer::ShowLidarProcessingResult(const LidarProcessingResult& result, bool save_intermediate) {
  if (!is_initialized_) {
    last_error_ = "Visualizer not initialized";
    return;
  }

  try {
    if (!result.success) {
      if (verbose_) {
        std::cout << "[Visualizer] LiDAR processing failed: " << result.error_message << std::endl;
      }
      return;
    }

    if (!config_.save_point_clouds) {
      if (verbose_) {
        std::cout << "[Visualizer] Point cloud saving is disabled" << std::endl;
      }
      return;
    }

    if (save_intermediate && config_.save_intermediate_results) {
      // 保存中间处理结果
      if (result.filtered_cloud && !result.filtered_cloud->empty()) {
        auto colored_filtered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        SetPointCloudColor(result.filtered_cloud, colored_filtered, kDefaultColors[0]);
        SaveColoredPointCloud(colored_filtered, "filtered_cloud.pcd");
      }

      if (result.plane_cloud && !result.plane_cloud->empty()) {
        auto colored_plane = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        SetPointCloudColor(result.plane_cloud, colored_plane, kDefaultColors[1]);
        SaveColoredPointCloud(colored_plane, "plane_cloud.pcd");
      }

      if (result.aligned_cloud && !result.aligned_cloud->empty()) {
        auto colored_aligned = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        SetPointCloudColor(result.aligned_cloud, colored_aligned, kDefaultColors[2]);
        SaveColoredPointCloud(colored_aligned, "aligned_cloud.pcd");
      }

      if (result.edge_cloud && !result.edge_cloud->empty()) {
        auto colored_edge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        SetPointCloudColor(result.edge_cloud, colored_edge, kDefaultColors[3]);
        SaveColoredPointCloud(colored_edge, "edge_cloud.pcd");
      }

      if (result.center_z0_cloud && !result.center_z0_cloud->empty()) {
        SaveCenterMarkers(result.center_z0_cloud, "centers_z0.pcd", kDefaultColors[4]);
      }
    }

    // 保存最终检测到的圆心
    if (result.circle_centers && !result.circle_centers->empty()) {
      SaveCenterMarkers(result.circle_centers, "final_centers.pcd", kDefaultColors[5]);
    }

    if (verbose_) {
      std::cout << "[Visualizer] LiDAR processing results saved. Processing time: " << result.processing_time_ms << "ms"
                << std::endl;
    }
  } catch (const std::exception& e) {
    last_error_ = "Error saving LiDAR processing result: " + std::string(e.what());
    if (verbose_) std::cout << "[Visualizer] " << last_error_ << std::endl;
  }
}

void Visualizer::ShowQrDetectionResult(const QRDetectionResult& result, bool save_processed_image) {
  if (!result.success) {
    if (verbose_) {
      std::cout << "[Visualizer] QR detection failed: " << result.error_message << std::endl;
    }
    return;
  }

  try {
    // 显示处理后的图像
    if (save_processed_image && !result.processed_image.empty()) {
      ShowImage(result.processed_image, "QR Detection Result", false);

      // 保存处理后的图像
      std::string image_filepath = BuildFilePath("qr_detection_result.png", "images");
      cv::imwrite(image_filepath, result.processed_image);

      if (verbose_) {
        std::cout << "[Visualizer] Saved QR detection image with " << result.markers_detected
                  << " markers detected to: " << image_filepath << std::endl;
      }
    }

    // 保存3D圆心坐标
    if (config_.save_point_clouds && result.circle_centers && !result.circle_centers->empty()) {
      SaveCenterMarkers(result.circle_centers, "qr_centers.pcd", kDefaultColors[6]);
      if (verbose_) {
        std::cout << "[Visualizer] Saved QR circle centers (" << result.circle_centers->size() << " points)"
                  << std::endl;
      }
    }
  } catch (const std::exception& e) {
    last_error_ = "Error saving QR detection result: " + std::string(e.what());
    if (verbose_) std::cout << "[Visualizer] " << last_error_ << std::endl;
  }
}

void Visualizer::ShowCalibrationResult(const RigidTransformResult& calib_result,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& original_cloud,
                                       const cv::Mat& camera_image, const cv::Mat& camera_matrix,
                                       const cv::Mat& distortion_coeffs) {
  if (!calib_result.success || !original_cloud || camera_image.empty()) {
    last_error_ = "Invalid calibration result or input data";
    return;
  }

  try {
    // 创建着色点云
    auto colored_cloud = CreateColoredPointCloud(original_cloud, calib_result.transformation, camera_image,
                                                 camera_matrix, distortion_coeffs);

    if (!colored_cloud || colored_cloud->empty()) {
      last_error_ = "Failed to create colored point cloud";
      return;
    }

    if (config_.save_point_clouds) {
      // 保存原始点云
      auto original_colored = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(original_cloud, original_colored, kDefaultColors[7]);
      SaveColoredPointCloud(original_colored, "original_cloud.pcd");

      // 保存标定后的着色点云
      SaveColoredPointCloud(colored_cloud, "calibrated_cloud.pcd");
    }

    // 显示并保存标定结果图像
    ShowImage(camera_image, "Camera Image", false);
    std::string image_filepath = BuildFilePath("camera_image.png", "images");
    cv::imwrite(image_filepath, camera_image);

    if (verbose_) {
      std::cout << "[Visualizer] Calibration result saved:" << std::endl;
      std::cout << "  RMSE: " << calib_result.rmse << " m" << std::endl;
      std::cout << "  Points used: " << calib_result.num_points << std::endl;
      std::cout << "  Colored cloud size: " << colored_cloud->size() << " points" << std::endl;
      std::cout << "  Camera image saved to: " << image_filepath << std::endl;
    }
  } catch (const std::exception& e) {
    last_error_ = "Error saving calibration result: " + std::string(e.what());
    if (verbose_) std::cout << "[Visualizer] " << last_error_ << std::endl;
  }
}

void Visualizer::ShowMultiSceneComparison(const std::vector<RigidTransformResult>& scene_results,
                                          const RigidTransformResult& final_result) {
  if (scene_results.empty() || !final_result.success) {
    last_error_ = "Invalid multi-scene comparison data";
    return;
  }

  try {
    if (verbose_) {
      std::cout << "[Visualizer] Multi-scene calibration comparison:" << std::endl;
      std::cout << "  Individual scenes: " << scene_results.size() << std::endl;
      std::cout << "  Final RMSE: " << final_result.rmse << " m" << std::endl;

      for (size_t i = 0; i < scene_results.size(); ++i) {
        std::cout << "  Scene " << i << " RMSE: " << scene_results[i].rmse << " m" << std::endl;
      }
    }

  } catch (const std::exception& e) {
    last_error_ = "Error showing multi-scene comparison: " + std::string(e.what());
    if (verbose_) std::cout << "[Visualizer] " << last_error_ << std::endl;
  }
}

void Visualizer::ShowImage(const cv::Mat& image, const std::string& window_name, bool wait_key) {
  if (image.empty()) {
    return;
  }

  try {
    cv::imshow(window_name, image);
    if (wait_key) {
      cv::waitKey(0);
      cv::destroyWindow(window_name);
    } else {
      cv::waitKey(1);  // 非阻塞更新
    }
  } catch (const std::exception& e) {
    last_error_ = "Error showing image: " + std::string(e.what());
  }
}

void Visualizer::SetVisualizationConfig(const VisualizationConfig& config) {
  config_ = config;
  if (is_initialized_) {
    ApplyOutputConfig();
  }
}

VisualizationConfig Visualizer::GetVisualizationConfig() const { return config_; }

void Visualizer::SetVerbose(bool verbose) { verbose_ = verbose; }

std::string Visualizer::GetLastError() const { return last_error_; }

// 私有方法实现

void Visualizer::InitializeOutputDirectory() {
  try {
    // 创建输出目录结构
    if (CreateOutputDirectories()) {
      ApplyOutputConfig();
      is_initialized_ = true;

      if (verbose_) {
        std::cout << "[Visualizer] Successfully initialized output directory: " << config_.output_directory
                  << std::endl;
      }
    } else {
      throw std::runtime_error("Failed to create output directories");
    }

  } catch (const std::exception& e) {
    last_error_ = "Failed to initialize output directory: " + std::string(e.what());
    is_initialized_ = false;

    if (verbose_) {
      std::cerr << "[Visualizer] " << last_error_ << std::endl;
      std::cerr << "[Visualizer] File saving will be disabled for this session" << std::endl;
    }
  }
}

void Visualizer::ApplyOutputConfig() {
  try {
    // 获取实际输出目录（包含数据集子目录）
    std::string actual_output_dir = GetActualOutputDirectory();

    // 确保输出目录存在
    if (!std::filesystem::exists(actual_output_dir)) {
      std::filesystem::create_directories(actual_output_dir);
    }

    // 只有在设置了数据集名称时才创建子目录
    // 避免在根 output 目录下创建空的 pointclouds 和 images 文件夹
    if (!config_.dataset_name.empty()) {
      std::filesystem::create_directories(actual_output_dir + "/pointclouds");
      std::filesystem::create_directories(actual_output_dir + "/images");

      if (verbose_) {
        std::cout << "[Visualizer] Output configuration applied successfully to: " << actual_output_dir << std::endl;
      }
    } else {
      // 数据集名称未设置时，只创建根目录，不创建子目录
      if (verbose_) {
        std::cout << "[Visualizer] Output directory created: " << actual_output_dir
                  << " (subdirectories will be created when dataset name is set)" << std::endl;
      }
    }
  } catch (const std::exception& e) {
    last_error_ = "Error applying output config: " + std::string(e.what());
  }
}

std::vector<double> Visualizer::ParseColorString(const std::string& color_str) const {
  std::vector<double> color(3, 0.0);

  try {
    std::stringstream ss(color_str);
    std::string token;
    int i = 0;

    while (std::getline(ss, token, ',') && i < 3) {
      color[i] = std::stod(token) / 255.0;  // 假设输入是0-255范围，转换为0-1
      ++i;
    }
  } catch (...) {
    // 解析失败，返回默认黑色
    color = {0.0, 0.0, 0.0};
  }

  return color;
}

std::string Visualizer::BuildFilePath(const std::string& filename, const std::string& subdirectory) const {
  // 使用实际输出目录（包含数据集子目录）
  std::string full_path = GetActualOutputDirectory();

  if (!subdirectory.empty()) {
    full_path += "/" + subdirectory;

    // 确保子目录存在，如果不存在则创建
    try {
      if (!std::filesystem::exists(full_path)) {
        std::filesystem::create_directories(full_path);
      }
    } catch (const std::exception& e) {
      // 记录错误但不中断，让文件保存尝试继续
      if (verbose_) {
        std::cerr << "[Visualizer] Warning: Failed to create subdirectory " << full_path << ": " << e.what()
                  << std::endl;
      }
    }
  }

  if (!config_.file_prefix.empty()) {
    full_path += "/" + config_.file_prefix + "_" + filename;
  } else {
    full_path += "/" + filename;
  }

  return full_path;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Visualizer::CreateColoredPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Matrix4d& transformation,
    const cv::Mat& camera_image, const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs) const {
  auto colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

  if (!cloud || cloud->empty() || camera_image.empty()) {
    return colored_cloud;
  }

  try {
    // 复用common_lib.h中的投影函数
    Eigen::Matrix4f transformation_f = transformation.cast<float>();
    ProjectPointCloudToImage(cloud, transformation_f, camera_matrix, distortion_coeffs, camera_image, colored_cloud);
  } catch (const std::exception& e) {
    // 投影失败，创建单色点云
    SetPointCloudColor(cloud, colored_cloud, {0.8, 0.8, 0.8});  // 灰色
  }

  return colored_cloud;
}

void Visualizer::SetPointCloudColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
                                    const std::vector<double>& color) const {
  if (!cloud || color.size() < 3) {
    return;
  }

  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  uint8_t r = static_cast<uint8_t>(color[0] * 255);
  uint8_t g = static_cast<uint8_t>(color[1] * 255);
  uint8_t b = static_cast<uint8_t>(color[2] * 255);

  for (const auto& pt : cloud->points) {
    pcl::PointXYZRGB colored_pt;
    colored_pt.x = pt.x;
    colored_pt.y = pt.y;
    colored_pt.z = pt.z;
    colored_pt.r = r;
    colored_pt.g = g;
    colored_pt.b = b;
    colored_cloud->push_back(colored_pt);
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Visualizer::CreateDummyPointCloud() const {
  auto dummy_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  // 创建一个简单的参考坐标系（原点附近的几个点）
  pcl::PointXYZI origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  origin.intensity = 100.0;
  dummy_cloud->points.push_back(origin);

  pcl::PointXYZI x_axis;
  x_axis.x = 1.0;
  x_axis.y = 0.0;
  x_axis.z = 0.0;
  x_axis.intensity = 100.0;
  dummy_cloud->points.push_back(x_axis);

  pcl::PointXYZI y_axis;
  y_axis.x = 0.0;
  y_axis.y = 1.0;
  y_axis.z = 0.0;
  y_axis.intensity = 100.0;
  dummy_cloud->points.push_back(y_axis);

  pcl::PointXYZI z_axis;
  z_axis.x = 0.0;
  z_axis.y = 0.0;
  z_axis.z = 1.0;
  z_axis.intensity = 100.0;
  dummy_cloud->points.push_back(z_axis);

  dummy_cloud->width = dummy_cloud->points.size();
  dummy_cloud->height = 1;
  dummy_cloud->is_dense = true;

  return dummy_cloud;
}

// 新增的文件保存方法实现

bool Visualizer::SavePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename) {
  if (!is_initialized_ || !cloud || cloud->empty()) {
    return false;
  }

  try {
    std::string filepath = BuildFilePath(filename, "pointclouds");
    int result = 0;

    if (config_.save_as_binary) {
      result = pcl::io::savePCDFileBinary(filepath, *cloud);
    } else {
      result = pcl::io::savePCDFileASCII(filepath, *cloud);
    }

    if (result == 0 && verbose_) {
      std::cout << "[Visualizer] Saved point cloud (" << cloud->size() << " points) to: " << filepath << std::endl;
    }

    return (result == 0);
  } catch (const std::exception& e) {
    last_error_ = "Error saving point cloud: " + std::string(e.what());
    return false;
  }
}

bool Visualizer::SaveColoredPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                       const std::string& filename) {
  if (!is_initialized_ || !cloud || cloud->empty()) {
    return false;
  }

  try {
    std::string filepath = BuildFilePath(filename, "pointclouds");
    int result = 0;

    if (config_.save_as_binary) {
      result = pcl::io::savePCDFileBinary(filepath, *cloud);
    } else {
      result = pcl::io::savePCDFileASCII(filepath, *cloud);
    }

    if (result == 0 && verbose_) {
      std::cout << "[Visualizer] Saved colored point cloud (" << cloud->size() << " points) to: " << filepath
                << std::endl;
    }

    return (result == 0);
  } catch (const std::exception& e) {
    last_error_ = "Error saving colored point cloud: " + std::string(e.what());
    return false;
  }
}

bool Visualizer::SaveCenterMarkers(const pcl::PointCloud<pcl::PointXYZI>::Ptr& centers, const std::string& filename,
                                   const std::vector<double>& color) {
  if (!is_initialized_ || !centers || centers->empty()) {
    return false;
  }

  try {
    // 将圆心转换为彩色点云
    auto colored_centers = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    SetPointCloudColor(centers, colored_centers, color);

    return SaveColoredPointCloud(colored_centers, filename);
  } catch (const std::exception& e) {
    last_error_ = "Error saving center markers: " + std::string(e.what());
    return false;
  }
}

bool Visualizer::CreateOutputDirectories() {
  try {
    // 创建主输出目录
    if (!std::filesystem::exists(config_.output_directory)) {
      std::filesystem::create_directories(config_.output_directory);
    }

    // 注意：pointclouds 和 images 子目录现在在 ApplyOutputConfig() 中
    // 根据数据集名称在实际输出目录下创建，而不是在根输出目录下

    return true;
  } catch (const std::exception& e) {
    last_error_ = "Error creating output directories: " + std::string(e.what());
    return false;
  }
}

void Visualizer::ClearOutputDirectory() {
  try {
    // 获取实际输出目录（包含数据集子目录）
    std::string actual_output_dir = GetActualOutputDirectory();

    if (!actual_output_dir.empty() && std::filesystem::exists(actual_output_dir)) {
      // 清理点云文件
      std::string pointcloud_dir = actual_output_dir + "/pointclouds";
      if (std::filesystem::exists(pointcloud_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(pointcloud_dir)) {
          if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
            std::filesystem::remove(entry.path());
          }
        }
      }

      // 清理图像文件
      std::string image_dir = actual_output_dir + "/images";
      if (std::filesystem::exists(image_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(image_dir)) {
          if (entry.is_regular_file()) {
            auto ext = entry.path().extension();
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
              std::filesystem::remove(entry.path());
            }
          }
        }
      }

      if (verbose_) {
        std::cout << "[Visualizer] Cleared output directory: " << actual_output_dir << std::endl;
      }
    }
  } catch (const std::exception& e) {
    last_error_ = "Error clearing output directory: " + std::string(e.what());
  }
}

// 失败诊断方法实现

void Visualizer::ShowFailedLidarProcessing(const LidarProcessingResult& result,
                                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& original_cloud) {
  if (!is_initialized_) {
    last_error_ = "Visualizer not initialized";
    return;
  }

  try {
    if (verbose_) {
      std::cout << "[Visualizer] Saving failed LiDAR processing diagnosis" << std::endl;
      std::cout << "[Visualizer] Error: " << result.error_message << std::endl;
    }

    if (!config_.save_point_clouds) {
      if (verbose_) {
        std::cout << "[Visualizer] Point cloud saving is disabled" << std::endl;
      }
      return;
    }

    // 保存原始点云
    if (original_cloud && !original_cloud->empty()) {
      auto colored_original = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(original_cloud, colored_original, kDefaultColors[0]);
      SaveColoredPointCloud(colored_original, "failed_original_cloud.pcd");
    }

    // 保存旋转后点云（用于验证 camera_direction / lidar_yaw_offset_deg 是否正确）
    if (result.rotated_cloud && !result.rotated_cloud->empty()) {
      auto colored_rotated = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(result.rotated_cloud, colored_rotated, kDefaultColors[1]);
      SaveColoredPointCloud(colored_rotated, "failed_rotated_cloud.pcd");
    }

    // 保存滤波后的点云（如果有）
    if (result.filtered_cloud && !result.filtered_cloud->empty()) {
      auto colored_filtered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(result.filtered_cloud, colored_filtered, kDefaultColors[1]);
      SaveColoredPointCloud(colored_filtered, "failed_filtered_cloud.pcd");
    }

    // 保存平面点云（如果有）
    if (result.plane_cloud && !result.plane_cloud->empty()) {
      auto colored_plane = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(result.plane_cloud, colored_plane, kDefaultColors[2]);
      SaveColoredPointCloud(colored_plane, "failed_plane_cloud.pcd");
    }

    // 保存对齐后的点云（如果有）
    if (result.aligned_cloud && !result.aligned_cloud->empty()) {
      auto colored_aligned = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(result.aligned_cloud, colored_aligned, kDefaultColors[3]);
      SaveColoredPointCloud(colored_aligned, "failed_aligned_cloud.pcd");
    }

    // 保存边界点云（如果有）
    if (result.edge_cloud && !result.edge_cloud->empty()) {
      auto colored_edge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(result.edge_cloud, colored_edge, kDefaultColors[4]);
      SaveColoredPointCloud(colored_edge, "failed_edge_cloud.pcd");
    }

    // 保存噪声点云（如果有）
    if (result.noise_cloud && !result.noise_cloud->empty()) {
      auto colored_noise = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      SetPointCloudColor(result.noise_cloud, colored_noise, kDefaultColors[5]);
      SaveColoredPointCloud(colored_noise, "failed_noise_cloud.pcd");
    }

  } catch (const std::exception& e) {
    last_error_ = "Failed to save LiDAR processing diagnosis: " + std::string(e.what());
    if (verbose_) {
      std::cerr << "[Visualizer] " << last_error_ << std::endl;
    }
  }
}

void Visualizer::ShowFailedQrDetection(const QRDetectionResult& result, const cv::Mat& original_image) {
  if (!is_initialized_) {
    last_error_ = "Visualizer not initialized";
    return;
  }

  try {
    if (verbose_) {
      std::cout << "[Visualizer] Saving failed QR detection diagnosis" << std::endl;
      std::cout << "[Visualizer] Error: " << result.error_message << std::endl;
    }

    // 保存原始图像
    if (!original_image.empty()) {
      cv::Mat display_image = original_image.clone();
      cv::putText(display_image, "QR Detection Failed", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar(0, 0, 255), 2);
      cv::putText(display_image, "Error: " + result.error_message, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 0, 255), 1);

      ShowImage(display_image, "Original Image (QR Detection Failed)", false);

      std::string image_filepath = BuildFilePath("failed_qr_original.png", "images");
      cv::imwrite(image_filepath, display_image);

      if (verbose_) {
        std::cout << "[Visualizer] Original image saved to: " << image_filepath << std::endl;
      }
    }

    // 保存处理后的图像（如果有）
    if (!result.processed_image.empty()) {
      cv::Mat processed_display = result.processed_image.clone();
      cv::putText(processed_display, "Partial Detection Result", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 0), 2);

      ShowImage(processed_display, "Processed Image (Partial Detection)", false);

      std::string processed_filepath = BuildFilePath("failed_qr_processed.png", "images");
      cv::imwrite(processed_filepath, processed_display);

      if (verbose_) {
        std::cout << "[Visualizer] Processed image saved to: " << processed_filepath << std::endl;
      }
    }

    // 保存点云数据（如果有）
    if (config_.save_point_clouds && result.circle_centers && !result.circle_centers->empty()) {
      SaveCenterMarkers(result.circle_centers, "failed_qr_partial_centers.pcd", {1.0, 0.0, 0.0});
      if (verbose_) {
        std::cout << "[Visualizer] Partial centers detected: " << result.circle_centers->size() << std::endl;
      }
    }

    // 确保OpenCV窗口更新
    cv::waitKey(1);

  } catch (const std::exception& e) {
    last_error_ = "Failed to save QR detection diagnosis: " + std::string(e.what());
    if (verbose_) {
      std::cerr << "[Visualizer] " << last_error_ << std::endl;
    }
  }
}

void Visualizer::ShowFailedCalibration(const QRDetectionResult& qr_result, const LidarProcessingResult& lidar_result,
                                       const cv::Mat& camera_image,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud,
                                       const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs) {
  if (!is_initialized_) {
    last_error_ = "Visualizer not initialized";
    return;
  }

  try {
    if (verbose_) {
      std::cout << "[Visualizer] Saving calibration failure diagnosis" << std::endl;
      std::cout << "[Visualizer] Analyzing input data..." << std::endl;
    }

    // 保存QR检测结果和原始图像
    if (!camera_image.empty()) {
      ShowImage(camera_image, "Camera Image with QR Detection", false);

      std::string camera_filepath = BuildFilePath("failed_calib_camera.png", "images");
      cv::imwrite(camera_filepath, camera_image);

      if (verbose_) {
        std::cout << "[Visualizer] Camera image saved to: " << camera_filepath << std::endl;
      }
    }

    if (!qr_result.processed_image.empty()) {
      ShowImage(qr_result.processed_image, "QR Processing Result", false);

      std::string qr_filepath = BuildFilePath("failed_calib_qr_result.png", "images");
      cv::imwrite(qr_filepath, qr_result.processed_image);

      if (verbose_) {
        std::cout << "[Visualizer] QR processing result saved to: " << qr_filepath << std::endl;
      }
    }

    if (config_.save_point_clouds) {
      // 保存原始点云
      if (point_cloud && !point_cloud->empty()) {
        auto colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        SetPointCloudColor(point_cloud, colored_cloud, {0.8, 0.8, 0.8});  // 灰色原始点云
        SaveColoredPointCloud(colored_cloud, "failed_calib_original_pointcloud.pcd");
      }

      // 保存LiDAR检测到的圆心
      if (lidar_result.circle_centers && !lidar_result.circle_centers->empty()) {
        SaveCenterMarkers(lidar_result.circle_centers, "failed_calib_lidar_centers.pcd", {0.0, 1.0, 0.0});  // 绿色
        if (verbose_) {
          std::cout << "[Visualizer] LiDAR centers: " << lidar_result.circle_centers->size() << std::endl;
        }
      }

      // 保存QR检测到的圆心
      if (qr_result.circle_centers && !qr_result.circle_centers->empty()) {
        SaveCenterMarkers(qr_result.circle_centers, "failed_calib_qr_centers.pcd", {1.0, 0.0, 0.0});  // 红色
        if (verbose_) {
          std::cout << "[Visualizer] QR centers: " << qr_result.circle_centers->size() << std::endl;
        }
      }
    }

    if (verbose_) {
      std::cout << "[Visualizer] Check if the number of detected centers match and are properly aligned" << std::endl;
    }

  } catch (const std::exception& e) {
    last_error_ = "Failed to save calibration diagnosis: " + std::string(e.what());
    if (verbose_) {
      std::cerr << "[Visualizer] " << last_error_ << std::endl;
    }
  }
}

std::string Visualizer::GetActualOutputDirectory() const {
  if (config_.dataset_name.empty()) {
    return config_.output_directory;
  } else {
    return config_.output_directory + "/" + config_.dataset_name;
  }
}