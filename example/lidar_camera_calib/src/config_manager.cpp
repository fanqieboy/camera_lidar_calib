/**
 * @file config_manager.cpp
 * @brief 配置管理系统实现
 */

#include "config_manager.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

ConfigManager::ConfigManager() : config_node_(std::make_unique<YAML::Node>()), loaded_(false) { SetDefaultParams(); }

ConfigManager::~ConfigManager() = default;

bool ConfigManager::LoadConfig(const std::string& config_path) {
  try {
    // 检查文件是否存在
    if (!std::filesystem::exists(config_path)) {
      last_error_ = "配置文件不存在: " + config_path;
      return false;
    }

    // 加载 YAML 文件
    *config_node_ = YAML::LoadFile(config_path);
    config_file_path_ = config_path;

    // 解析配置节点
    if (!ParseYamlNode(*config_node_)) {
      return false;
    }

    // 验证参数
    if (!ValidateParams()) {
      return false;
    }

    loaded_ = true;
    last_error_.clear();

    std::cout << "[ConfigManager] 成功加载配置文件: " << config_path << std::endl;
    PrintParams();
    return true;
  } catch (const YAML::Exception& e) {
    last_error_ = "YAML 解析错误: " + std::string(e.what());
    return false;
  } catch (const std::exception& e) {
    last_error_ = "配置加载错误: " + std::string(e.what());
    return false;
  }
}

bool ConfigManager::LoadDefaultConfig() {
  SetDefaultParams();
  loaded_ = true;
  last_error_.clear();

  std::cout << "[ConfigManager] 使用默认配置参数" << std::endl;
  return true;
}

const Params& ConfigManager::GetParams() const { return params_; }

bool ConfigManager::ValidateParams() const {
  // 验证标定板参数
  if (!ValidateRange(params_.marker_size, 0.01, 1.0, "marker_size")) return false;
  if (!ValidateRange(params_.circle_radius, 0.01, 1.0, "circle_radius")) return false;

  if (!ValidateRange(params_.delta_width_qr_center, 0.1, 2.0, "delta_width_qr_center")) return false;
  if (!ValidateRange(params_.delta_height_qr_center, 0.1, 2.0, "delta_height_qr_center")) return false;
  if (!ValidateRange(params_.delta_width_circles, 0.1, 2.0, "delta_width_circles")) return false;
  if (!ValidateRange(params_.delta_height_circles, 0.1, 2.0, "delta_height_circles")) return false;

  // 验证点云滤波范围
  if (params_.x_min >= params_.x_max) {
    last_error_ = "参数错误: x_min 必须小于 x_max";
    return false;
  }
  if (params_.y_min >= params_.y_max) {
    last_error_ = "参数错误: y_min 必须小于 y_max";
    return false;
  }
  if (params_.z_min >= params_.z_max) {
    last_error_ = "参数错误: z_min 必须小于 z_max";
    return false;
  }

  // 验证检测参数
  if (params_.min_detected_markers < 1 || params_.min_detected_markers > 10) {
    last_error_ = "参数错误: min_detected_markers 必须在 1-10 之间";
    return false;
  }

  // 验证新数据集参数
  if (params_.max_pcds != -1 && params_.max_pcds <= 0) {
    last_error_ = "参数错误: max_pcds 必须是正数或 -1";
    return false;
  }

  // 验证日志级别
  if (params_.log_level != "DEBUG" && params_.log_level != "INFO" && params_.log_level != "WARN" &&
      params_.log_level != "ERROR") {
    last_error_ = "参数错误: log_level 必须是 'DEBUG', 'INFO', 'WARN' 或 'ERROR'";
    return false;
  }

  // 验证相机方向参数
  if (params_.camera_direction < 0 || params_.camera_direction > 3) {
    last_error_ = "参数错误: camera_direction 必须在 0-3 之间 (0:正前方,1:背后,2:左侧,3:右侧)";
    return false;
  }

  return true;
}

void ConfigManager::PrintParams() const {
  std::cout << "\n=== 配置参数 ===" << std::endl;
  std::cout << "标定板参数:" << std::endl;
  std::cout << "  marker_size: " << params_.marker_size << std::endl;
  std::cout << "  circle_radius: " << params_.circle_radius << std::endl;
  std::cout << "  delta_width_qr_center: " << params_.delta_width_qr_center << std::endl;
  std::cout << "  delta_height_qr_center: " << params_.delta_height_qr_center << std::endl;
  std::cout << "点云滤波范围:" << std::endl;
  std::cout << "  x: [" << params_.x_min << ", " << params_.x_max << "]" << std::endl;
  std::cout << "  y: [" << params_.y_min << ", " << params_.y_max << "]" << std::endl;
  std::cout << "  z: [" << params_.z_min << ", " << params_.z_max << "]" << std::endl;
  std::cout << "文件路径:" << std::endl;
  std::cout << "  image_path: " << params_.image_path << std::endl;
  std::cout << "  output_path: " << params_.output_path << std::endl;
  std::cout << "  camera_model_path: " << params_.camera_model_path << std::endl;
  std::cout << "数据集参数:" << std::endl;
  std::cout << "  dataset_dir: " << params_.dataset_dir << std::endl;
  std::cout << "  camera_id: " << params_.camera_id << std::endl;
  std::cout << "  max_pcds: " << params_.max_pcds << std::endl;
  std::cout << "其他参数:" << std::endl;
  std::cout << "  enable_visualization: " << (params_.enable_visualization ? "true" : "false") << std::endl;
  std::cout << "  save_intermediate_results: " << (params_.save_intermediate_results ? "true" : "false") << std::endl;
  std::cout << "  show_failed_results: " << (params_.show_failed_results ? "true" : "false") << std::endl;
  std::cout << "  continue_on_partial_failure: " << (params_.continue_on_partial_failure ? "true" : "false")
            << std::endl;
  std::cout << "  log_level: " << params_.log_level << std::endl;
  std::cout << "去畸变预处理参数:" << std::endl;
  std::cout << "  output_dataset_dir: " << params_.output_dataset_dir << std::endl;
  std::cout << "  preserve_original_data: " << (params_.preserve_original_data ? "true" : "false") << std::endl;
  std::cout << "相机方向参数:" << std::endl;
  std::cout << "  camera_direction: " << params_.camera_direction << std::endl;
  std::cout << "  lidar_yaw_offset_deg: " << params_.lidar_yaw_offset_deg << std::endl;
  std::cout << "=================" << std::endl;
}

std::string ConfigManager::GetLastError() const { return last_error_; }

bool ConfigManager::IsLoaded() const { return loaded_; }

void ConfigManager::SetDefaultParams() {
  // 相机内参 (基于原 qr_params.yaml 的黑白相机参数)
  params_.fx = 0.;
  params_.fy = 0.;
  params_.cx = 0.;
  params_.cy = 0.;

  // 标定板参数
  params_.marker_size = 0.16;
  params_.delta_width_qr_center = 0.55;
  params_.delta_height_qr_center = 0.35;
  params_.delta_width_circles = 0.5;
  params_.delta_height_circles = 0.4;
  params_.circle_radius = 0.12;
  params_.min_detected_markers = 3;

  // 点云滤波范围
  params_.x_min = 1.5;
  params_.x_max = 3.0;
  params_.y_min = -1.5;
  params_.y_max = 2.0;
  params_.z_min = -0.5;
  params_.z_max = 2.0;

  // 文件路径
  params_.image_path = "./data/image.png";
  // 保留兼容性，但独立版本不使用
  params_.bag_path = "./data/input.bag";
  params_.lidar_topic = "/livox/lidar";
  params_.output_path = "./output";

  // 新数据集支持参数 (Feature 001-1-pcd-pcd)
  params_.dataset_dir = "";
  params_.camera_id = "";
  params_.max_pcds = -1;

  // 可选参数 (独立版本特有)
  params_.enable_visualization = true;
  params_.save_intermediate_results = false;
  params_.show_failed_results = true;
  params_.continue_on_partial_failure = true;
  params_.log_level = "INFO";

  // 去畸变预处理参数
  params_.output_dataset_dir = "";
  params_.preserve_original_data = true;

  // 相机方向参数
  params_.camera_direction = 0;       // 默认为正前方
  params_.lidar_yaw_offset_deg = 0.0; // 默认无附加偏差
}

bool ConfigManager::ParseYamlNode(const YAML::Node& config) {
  try {
    // 解析相机内参
    if (config["fx"]) params_.fx = config["fx"].as<double>();
    if (config["fy"]) params_.fy = config["fy"].as<double>();
    if (config["cx"]) params_.cx = config["cx"].as<double>();
    if (config["cy"]) params_.cy = config["cy"].as<double>();

    // 解析标定板参数
    if (config["marker_size"]) params_.marker_size = config["marker_size"].as<double>();
    if (config["delta_width_qr_center"]) params_.delta_width_qr_center = config["delta_width_qr_center"].as<double>();
    if (config["delta_height_qr_center"])
      params_.delta_height_qr_center = config["delta_height_qr_center"].as<double>();
    if (config["delta_width_circles"]) params_.delta_width_circles = config["delta_width_circles"].as<double>();
    if (config["delta_height_circles"]) params_.delta_height_circles = config["delta_height_circles"].as<double>();
    if (config["circle_radius"]) params_.circle_radius = config["circle_radius"].as<double>();
    if (config["min_detected_markers"]) params_.min_detected_markers = config["min_detected_markers"].as<int>();

    // 解析滤波范围
    if (config["x_min"]) params_.x_min = config["x_min"].as<double>();
    if (config["x_max"]) params_.x_max = config["x_max"].as<double>();
    if (config["y_min"]) params_.y_min = config["y_min"].as<double>();
    if (config["y_max"]) params_.y_max = config["y_max"].as<double>();
    if (config["z_min"]) params_.z_min = config["z_min"].as<double>();
    if (config["z_max"]) params_.z_max = config["z_max"].as<double>();

    // 解析文件路径
    if (config["image_path"]) params_.image_path = config["image_path"].as<std::string>();
    if (config["bag_path"]) params_.bag_path = config["bag_path"].as<std::string>();
    if (config["lidar_topic"]) params_.lidar_topic = config["lidar_topic"].as<std::string>();
    if (config["output_path"]) params_.output_path = config["output_path"].as<std::string>();
    if (config["camera_model_path"]) params_.camera_model_path = config["camera_model_path"].as<std::string>();

    // 解析新数据集支持参数 (Feature 001-1-pcd-pcd)
    if (config["dataset_dir"]) params_.dataset_dir = config["dataset_dir"].as<std::string>();
    if (config["camera_id"]) params_.camera_id = config["camera_id"].as<std::string>();
    if (config["max_pcds"]) params_.max_pcds = config["max_pcds"].as<int>();

    // 解析可选参数 (独立版本特有)
    if (config["enable_visualization"]) params_.enable_visualization = config["enable_visualization"].as<bool>();
    if (config["save_intermediate_results"])
      params_.save_intermediate_results = config["save_intermediate_results"].as<bool>();
    if (config["show_failed_results"]) params_.show_failed_results = config["show_failed_results"].as<bool>();
    if (config["continue_on_partial_failure"])
      params_.continue_on_partial_failure = config["continue_on_partial_failure"].as<bool>();
    if (config["log_level"]) params_.log_level = config["log_level"].as<std::string>();

    // 解析去畸变预处理参数
    if (config["output_dataset_dir"]) params_.output_dataset_dir = config["output_dataset_dir"].as<std::string>();
    if (config["preserve_original_data"]) params_.preserve_original_data = config["preserve_original_data"].as<bool>();

    // 解析相机方向参数
    if (config["camera_direction"]) params_.camera_direction = config["camera_direction"].as<int>();
    if (config["lidar_yaw_offset_deg"]) params_.lidar_yaw_offset_deg = config["lidar_yaw_offset_deg"].as<double>();

    return true;
  } catch (const YAML::Exception& e) {
    last_error_ = "YAML 节点解析错误: " + std::string(e.what());
    return false;
  }
}

bool ConfigManager::ValidatePath(const std::string& path, const std::string& param_name) const {
  if (path.empty()) {
    last_error_ = "参数错误: " + param_name + " 不能为空";
    return false;
  }

  // 对于输出路径，检查父目录是否存在
  if (param_name == "output_path") {
    std::filesystem::path parent_path = std::filesystem::path(path).parent_path();
    if (!parent_path.empty() && !std::filesystem::exists(parent_path)) {
      last_error_ = "参数错误: " + param_name + " 的父目录不存在: " + parent_path.string();
      return false;
    }
  }

  return true;
}

bool ConfigManager::ValidateRange(double value, double min_val, double max_val, const std::string& param_name) const {
  if (std::isnan(value) || std::isinf(value)) {
    last_error_ = "参数错误: " + param_name + " 不是有效数值";
    return false;
  }

  if (value < min_val || value > max_val) {
    last_error_ =
        "参数错误: " + param_name + " 超出有效范围 [" + std::to_string(min_val) + ", " + std::to_string(max_val) + "]";
    return false;
  }

  return true;
}

// 模板方法特化实现
template <>
double ConfigManager::GetParam<double>(const std::string& key, const double& default_value) const {
  if (!loaded_ || !config_node_) return default_value;

  try {
    if ((*config_node_)[key]) {
      return (*config_node_)[key].as<double>();
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "[ConfigManager] 获取参数错误 " << key << ": " << e.what() << std::endl;
  }

  return default_value;
}

template <>
int ConfigManager::GetParam<int>(const std::string& key, const int& default_value) const {
  if (!loaded_ || !config_node_) return default_value;

  try {
    if ((*config_node_)[key]) {
      return (*config_node_)[key].as<int>();
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "[ConfigManager] 获取参数错误 " << key << ": " << e.what() << std::endl;
  }

  return default_value;
}

template <>
std::string ConfigManager::GetParam<std::string>(const std::string& key, const std::string& default_value) const {
  if (!loaded_ || !config_node_) return default_value;

  try {
    if ((*config_node_)[key]) {
      return (*config_node_)[key].as<std::string>();
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "[ConfigManager] 获取参数错误 " << key << ": " << e.what() << std::endl;
  }

  return default_value;
}

template <>
bool ConfigManager::GetParam<bool>(const std::string& key, const bool& default_value) const {
  if (!loaded_ || !config_node_) return default_value;

  try {
    if ((*config_node_)[key]) {
      return (*config_node_)[key].as<bool>();
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "[ConfigManager] 获取参数错误 " << key << ": " << e.what() << std::endl;
  }

  return default_value;
}