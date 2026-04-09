/**
 * @file data_loader.cpp
 * @brief 数据加载系统实现文件 - 替代ROS bag和消息类型
 * @author LiDAR-Camera-Calib ROS-Decoupling Project
 * @date 2024-01-01
 * @version 1.0.0
 *
 * 该文件实现了独立的数据加载系统，提供标准文件格式的数据输入功能。
 */

#include "data_loader.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <sstream>

#include <pcl/io/pcd_io.h>

#include "config_manager.h"

// 定义支持的格式
const std::vector<std::string> DataLoader::kSupportedImageFormats = {".png", ".PNG", ".jpg",  ".JPG",  ".jpeg", ".JPEG",
                                                                     ".bmp", ".BMP", ".tiff", ".TIFF", ".tif",  ".TIF"};

const std::vector<std::string> DataLoader::kSupportedPointCloudFormats = {".pcd", ".PCD"};

DataLoader::DataLoader()
    : verbose_(false), images_loaded_(0), pointclouds_loaded_(0), images_failed_(0), pointclouds_failed_(0) {}

DataLoader::~DataLoader() = default;

cv::Mat DataLoader::LoadImage(const std::string& image_path, int flags) {
  if (!ValidateFile(image_path)) {
    last_error_ = "图像文件不存在或无法访问: " + image_path;
    UpdateStats(false, "image");
    return cv::Mat();
  }

  if (!IsValidImageFormat(image_path)) {
    last_error_ = "不支持的图像格式: " + GetFileExtension(image_path);
    UpdateStats(false, "image");
    return cv::Mat();
  }

  try {
    cv::Mat image = cv::imread(image_path, flags);

    if (image.empty()) {
      last_error_ = "图像加载失败: " + image_path;
      UpdateStats(false, "image");
      return cv::Mat();
    }

    UpdateStats(true, "image");

    if (verbose_) {
      std::cout << "[DataLoader] 成功加载图像: " << image_path << " (尺寸: " << image.cols << "x" << image.rows
                << ", 通道: " << image.channels() << ")" << std::endl;
    }

    return image;
  } catch (const cv::Exception& e) {
    last_error_ = "OpenCV 错误: " + std::string(e.what());
    UpdateStats(false, "image");
    return cv::Mat();
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DataLoader::LoadPointCloud(const std::string& pcd_path) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (!ValidateFile(pcd_path)) {
    last_error_ = "点云文件不存在或无法访问: " + pcd_path;
    UpdateStats(false, "pointcloud");
    return nullptr;
  }

  if (!IsValidPointCloudFormat(pcd_path)) {
    last_error_ = "不支持的点云格式: " + GetFileExtension(pcd_path);
    UpdateStats(false, "pointcloud");
    return nullptr;
  }

  try {
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1) {
      last_error_ = "点云加载失败: " + pcd_path;
      UpdateStats(false, "pointcloud");
      return nullptr;
    }

    UpdateStats(true, "pointcloud");

    if (verbose_) {
      std::cout << "[DataLoader] 成功加载点云: " << pcd_path << " (点数: " << cloud->size() << ")" << std::endl;
    }

    return cloud;
  } catch (const std::exception& e) {
    last_error_ = "PCL 错误: " + std::string(e.what());
    UpdateStats(false, "pointcloud");
    return nullptr;
  }
}

SceneData DataLoader::LoadSingleScene(const Params& params) {
  SceneData scene_data;

  // 设置场景名称
  scene_data.scene_name = "single_scene";
  scene_data.image_path = params.image_path;

  // 注意：独立版本不使用 ROS bag，而是使用 pcd_path 指定的点云文件
  // 用户应该提供 pcd_path 参数或通过其他方式指定点云路径

  // 加载图像
  scene_data.image = LoadImage(params.image_path);
  if (scene_data.image.empty()) {
    scene_data.is_valid = false;
    return scene_data;
  }

  // 为了向后兼容，我们假设点云文件与图像在同一目录
  // 用户需要提供对应的 .pcd 文件
  std::filesystem::path image_path(params.image_path);
  std::string pcd_path = image_path.parent_path().string() + "/" + image_path.stem().string() + ".pcd";

  scene_data.pointcloud_path = pcd_path;
  scene_data.point_cloud = LoadPointCloud(pcd_path);

  if (!scene_data.point_cloud || scene_data.point_cloud->empty()) {
    scene_data.is_valid = false;
    if (verbose_) {
      std::cout << "[DataLoader] 警告: 无法加载对应的点云文件: " << pcd_path << std::endl;
      std::cout << "[DataLoader] 请确保点云文件存在且命名正确" << std::endl;
    }
    return scene_data;
  }

  scene_data.is_valid = true;

  if (verbose_) {
    std::cout << "[DataLoader] 成功加载单场景数据:" << std::endl;
    std::cout << "  图像: " << scene_data.image_path << std::endl;
    std::cout << "  点云: " << scene_data.pointcloud_path << std::endl;
  }

  return scene_data;
}

std::vector<SceneData> DataLoader::LoadMultiSceneData(
    const std::vector<std::pair<std::string, std::string>>& scene_configs) {
  std::vector<SceneData> scenes;
  scenes.reserve(scene_configs.size());

  for (size_t i = 0; i < scene_configs.size(); ++i) {
    const auto& config = scene_configs[i];
    SceneData scene_data;

    scene_data.scene_name = "scene_" + std::to_string(i);
    scene_data.image_path = config.first;
    scene_data.pointcloud_path = config.second;

    // 加载图像
    scene_data.image = LoadImage(config.first);
    if (scene_data.image.empty()) {
      if (verbose_) {
        std::cout << "[DataLoader] 跳过场景 " << i << ": 图像加载失败" << std::endl;
      }
      continue;
    }

    // 加载点云
    scene_data.point_cloud = LoadPointCloud(config.second);
    if (!scene_data.point_cloud || scene_data.point_cloud->empty()) {
      if (verbose_) {
        std::cout << "[DataLoader] 跳过场景 " << i << ": 点云加载失败" << std::endl;
      }
      continue;
    }

    scene_data.is_valid = true;
    scenes.push_back(scene_data);

    if (verbose_) {
      std::cout << "[DataLoader] 成功加载场景 " << i << std::endl;
    }
  }

  if (verbose_) {
    std::cout << "[DataLoader] 多场景加载完成: " << scenes.size() << "/" << scene_configs.size() << " 场景成功"
              << std::endl;
  }

  return scenes;
}

std::vector<SceneData> DataLoader::LoadBatchData(const std::string& data_directory, const std::string& image_extension,
                                                 const std::string& pointcloud_extension) {
  std::vector<SceneData> scenes;

  if (!std::filesystem::exists(data_directory)) {
    last_error_ = "数据目录不存在: " + data_directory;
    return scenes;
  }

  std::vector<std::string> image_files;
  std::vector<std::string> pcd_files;

  // 扫描目录中的文件
  try {
    for (const auto& entry : std::filesystem::directory_iterator(data_directory)) {
      if (entry.is_regular_file()) {
        std::string file_path = entry.path().string();
        std::string extension = GetFileExtension(file_path);

        if (extension == image_extension) {
          image_files.push_back(file_path);
        } else if (extension == pointcloud_extension) {
          pcd_files.push_back(file_path);
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    last_error_ = "目录扫描失败: " + std::string(e.what());
    return scenes;
  }

  // 排序以确保匹配顺序
  std::sort(image_files.begin(), image_files.end());
  std::sort(pcd_files.begin(), pcd_files.end());

  // 根据文件名匹配图像和点云
  for (const auto& image_file : image_files) {
    std::string base_name = ExtractFileName(image_file);

    // 查找对应的点云文件
    std::filesystem::path data_path(data_directory);
    std::string expected_pcd = (data_path / (base_name + pointcloud_extension)).string();

    if (std::find(pcd_files.begin(), pcd_files.end(), expected_pcd) != pcd_files.end()) {
      SceneData scene_data;
      scene_data.scene_name = base_name;
      scene_data.image_path = image_file;
      scene_data.pointcloud_path = expected_pcd;

      // 加载数据
      scene_data.image = LoadImage(image_file);
      scene_data.point_cloud = LoadPointCloud(expected_pcd);

      if (!scene_data.image.empty() && scene_data.point_cloud && !scene_data.point_cloud->empty()) {
        scene_data.is_valid = true;
        scenes.push_back(scene_data);
      }
    }
  }

  if (verbose_) {
    std::cout << "[DataLoader] 批量加载完成: " << scenes.size() << " 个有效场景" << std::endl;
  }

  return scenes;
}

bool DataLoader::IsValidImageFormat(const std::string& image_path) {
  std::string extension = std::filesystem::path(image_path).extension().string();
  return std::find(kSupportedImageFormats.begin(), kSupportedImageFormats.end(), extension) !=
         kSupportedImageFormats.end();
}

bool DataLoader::IsValidPointCloudFormat(const std::string& pcd_path) {
  std::string extension = std::filesystem::path(pcd_path).extension().string();
  return std::find(kSupportedPointCloudFormats.begin(), kSupportedPointCloudFormats.end(), extension) !=
         kSupportedPointCloudFormats.end();
}

std::vector<std::string> DataLoader::GetSupportedImageFormats() { return kSupportedImageFormats; }

std::vector<std::string> DataLoader::GetSupportedPointCloudFormats() { return kSupportedPointCloudFormats; }

std::string DataLoader::GetLastError() const { return last_error_; }

std::string DataLoader::GetLoadingStats() const {
  std::ostringstream stats_stream;
  stats_stream << "加载统计 - 图像: " << images_loaded_ << " 成功, " << images_failed_
               << " 失败; 点云: " << pointclouds_loaded_ << " 成功, " << pointclouds_failed_ << " 失败";
  return stats_stream.str();
}

void DataLoader::ClearStatus() {
  last_error_.clear();
  images_loaded_ = 0;
  pointclouds_loaded_ = 0;
  images_failed_ = 0;
  pointclouds_failed_ = 0;
}

void DataLoader::SetVerbose(bool verbose) { verbose_ = verbose; }

bool DataLoader::ValidateFile(const std::string& file_path) {
  return std::filesystem::exists(file_path) && std::filesystem::is_regular_file(file_path);
}

std::string DataLoader::ExtractFileName(const std::string& file_path) {
  return std::filesystem::path(file_path).stem().string();
}

std::string DataLoader::GetFileExtension(const std::string& file_path) {
  return std::filesystem::path(file_path).extension().string();
}

void DataLoader::UpdateStats(bool success, const std::string& file_type) {
  static const std::string kImageType = "image";
  static const std::string kPointCloudType = "pointcloud";

  if (file_type == kImageType) {
    if (success) {
      ++images_loaded_;
    } else {
      ++images_failed_;
    }
  } else if (file_type == kPointCloudType) {
    if (success) {
      ++pointclouds_loaded_;
    } else {
      ++pointclouds_failed_;
    }
  }
}