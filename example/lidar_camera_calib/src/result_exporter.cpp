/**
 * @file result_exporter.cpp
 * @brief 结果导出模块实现 - 多格式标定结果导出
 */

#include "result_exporter.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>

#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>

#include "calib_core.h"
#include "common_lib.h"
#include "lidar_detector.h"
#include "qr_detector.h"
#include "visualizer.h"

// 静态映射定义
const std::map<OutputFormat, std::string> ResultExporter::kFormatExtensions = {{OutputFormat::kTxt, ".txt"},
                                                                               {OutputFormat::kYaml, ".yaml"},
                                                                               {OutputFormat::kJson, ".json"},
                                                                               {OutputFormat::kXml, ".xml"},
                                                                               {OutputFormat::kCsv, ".csv"}};

const std::map<std::string, OutputFormat> ResultExporter::kExtensionFormats = {
    {".txt", OutputFormat::kTxt},   {".yaml", OutputFormat::kYaml}, {".yml", OutputFormat::kYaml},
    {".json", OutputFormat::kJson}, {".xml", OutputFormat::kXml},   {".csv", OutputFormat::kCsv}};

ResultExporter::ResultExporter(const ExportConfig& config)
    : config_(config), verbose_(false), total_exports_(0), successful_exports_(0) {
  // 初始化格式计数器
  for (auto format : config_.enabled_formats) {
    format_counts_[format] = 0;
  }

  // 确保输出目录存在
  EnsureOutputDirectoryExists();

  if (verbose_) {
    std::cout << "[ResultExporter] Initialized with output directory: " << config_.output_directory << std::endl;
    std::cout << "[ResultExporter] Enabled formats: ";
    for (auto format : config_.enabled_formats) {
      std::cout << GetFormatExtension(format) << " ";
    }
    std::cout << std::endl;
  }
}

ResultExporter::~ResultExporter() {
  if (verbose_ && total_exports_ > 0) {
    std::cout << "[ResultExporter] Export statistics: " << successful_exports_ << "/" << total_exports_
              << " successful exports" << std::endl;
  }
}

bool ResultExporter::ExportSingleSceneResult(const CalibrationResultPackage& result_package,
                                             const Params& camera_params) {
  total_exports_++;

  if (!result_package.transformation_result.success) {
    last_error_ = "Cannot export failed calibration result";
    return false;
  }

  // 对于单场景标定，设置数据集子目录
  if (result_package.scene_name == "single_scene" && config_.dataset_name.empty()) {
    std::string dataset_name = ExtractDatasetName(camera_params);
    config_.dataset_name = dataset_name;

    if (verbose_) {
      std::cout << "[ResultExporter] Using dataset subdirectory: " << dataset_name << std::endl;
    }
  }

  // 确保实际输出目录存在（包含数据集子目录）
  std::string actual_output_dir = GetActualOutputDirectory();
  if (!CreateDirectory(actual_output_dir)) {
    last_error_ = "Failed to create output directory: " + actual_output_dir;
    return false;
  }

  bool overall_success = true;

  try {
    // 为每种启用的格式导出结果
    for (auto format : config_.enabled_formats) {
      std::string base_filename = config_.filename_prefix;
      if (!result_package.scene_name.empty()) {
        base_filename += "_" + result_package.scene_name;
      }

      std::string filepath = GenerateOutputPath(base_filename, format);
      bool format_success = false;

      switch (format) {
        case OutputFormat::kTxt:
          format_success = ExportAsTxt(result_package, camera_params, filepath);
          break;
        case OutputFormat::kYaml:
          format_success = ExportAsYaml(result_package, camera_params, filepath);
          break;
        case OutputFormat::kJson:
          format_success = ExportAsJson(result_package, camera_params, filepath);
          break;
        case OutputFormat::kXml:
          format_success = ExportAsXml(result_package, camera_params, filepath);
          break;
        default:
          format_success = false;
          break;
      }

      if (format_success) {
        format_counts_[format]++;
        if (verbose_) {
          std::cout << "[ResultExporter] Exported " << GetFormatExtension(format) << " format to: " << filepath
                    << std::endl;
        }
      } else {
        overall_success = false;
        if (verbose_) {
          std::cout << "[ResultExporter] Failed to export " << GetFormatExtension(format) << " format" << std::endl;
        }
      }
    }

    // 导出点云文件
    if (config_.save_point_clouds) {
      if (result_package.colored_cloud && !result_package.colored_cloud->empty()) {
        std::string cloud_filename = config_.filename_prefix + "_colored_cloud";
        if (!result_package.scene_name.empty()) {
          cloud_filename += "_" + result_package.scene_name;
        }
        ExportColoredPointCloud(result_package.colored_cloud, cloud_filename, !config_.compress_point_clouds);
      }

      if (result_package.lidar_centers && !result_package.lidar_centers->empty()) {
        std::string centers_filename = config_.filename_prefix + "_lidar_centers";
        if (!result_package.scene_name.empty()) {
          centers_filename += "_" + result_package.scene_name;
        }
        ExportPointCloud(result_package.lidar_centers, centers_filename, true);
      }

      if (result_package.camera_centers && !result_package.camera_centers->empty()) {
        std::string centers_filename = config_.filename_prefix + "_camera_centers";
        if (!result_package.scene_name.empty()) {
          centers_filename += "_" + result_package.scene_name;
        }
        ExportPointCloud(result_package.camera_centers, centers_filename, true);
      }
    }

    // 导出图像文件
    if (config_.save_images) {
      if (!result_package.camera_image.empty()) {
        std::string image_filename = config_.filename_prefix + "_camera_image";
        if (!result_package.scene_name.empty()) {
          image_filename += "_" + result_package.scene_name;
        }
        ExportImage(result_package.camera_image, image_filename, "png");
      }

      if (!result_package.processed_image.empty()) {
        std::string processed_filename = config_.filename_prefix + "_processed_image";
        if (!result_package.scene_name.empty()) {
          processed_filename += "_" + result_package.scene_name;
        }
        ExportImage(result_package.processed_image, processed_filename, "png");
      }
    }
  } catch (const std::exception& e) {
    last_error_ = "Export failed: " + std::string(e.what());
    overall_success = false;
  }

  if (overall_success) {
    successful_exports_++;
  }

  return overall_success;
}

bool ResultExporter::ExportMultiSceneResult(const std::vector<CalibrationResultPackage>& individual_results,
                                            const CalibrationResultPackage& final_result, const Params& camera_params) {
  bool success = true;

  // 导出最终结果
  CalibrationResultPackage final_package = final_result;
  final_package.scene_name = "multi_scene_final";

  // 导出专门的多场景结果文件（与原始ROS版本格式匹配）
  if (!ExportMultiSceneResultOriginalFormat(individual_results, final_result)) {
    success = false;
  }

  // 导出各个场景的结果
  if (config_.save_intermediate_results) {
    for (size_t i = 0; i < individual_results.size(); ++i) {
      CalibrationResultPackage scene_package = individual_results[i];
      if (scene_package.scene_name.empty()) {
        scene_package.scene_name = "scene_" + std::to_string(i);
      }

      if (!ExportSingleSceneResult(scene_package, camera_params)) {
        success = false;
      }
    }
  }

  // 导出统计报告
  if (!ExportCalibrationReport(individual_results, "multi_scene_report")) {
    success = false;
  }

  return success;
}

bool ResultExporter::ExportLidarProcessingResult(const LidarProcessingResult& lidar_result,
                                                 const std::string& scene_name) {
  if (!lidar_result.success || !config_.save_intermediate_results) {
    return true;  // 不是错误，只是不需要保存
  }

  bool success = true;
  std::string base_name = "lidar_processing";
  if (!scene_name.empty()) {
    base_name += "_" + scene_name;
  }

  try {
    // 保存各个中间结果点云
    if (lidar_result.filtered_cloud && !lidar_result.filtered_cloud->empty()) {
      ExportPointCloud(lidar_result.filtered_cloud, base_name + "_filtered", true);
    }

    if (lidar_result.plane_cloud && !lidar_result.plane_cloud->empty()) {
      ExportPointCloud(lidar_result.plane_cloud, base_name + "_plane", true);
    }

    if (lidar_result.aligned_cloud && !lidar_result.aligned_cloud->empty()) {
      ExportPointCloud(lidar_result.aligned_cloud, base_name + "_aligned", true);
    }

    if (lidar_result.edge_cloud && !lidar_result.edge_cloud->empty()) {
      ExportPointCloud(lidar_result.edge_cloud, base_name + "_edge", true);
    }

    if (lidar_result.center_z0_cloud && !lidar_result.center_z0_cloud->empty()) {
      ExportPointCloud(lidar_result.center_z0_cloud, base_name + "_centers_z0", true);
    }

    if (lidar_result.circle_centers && !lidar_result.circle_centers->empty()) {
      ExportPointCloud(lidar_result.circle_centers, base_name + "_final_centers", true);
    }

    if (verbose_) {
      std::cout << "[ResultExporter] Exported LiDAR processing intermediate results for: " << scene_name << std::endl;
    }
  } catch (const std::exception& e) {
    last_error_ = "Failed to export LiDAR processing result: " + std::string(e.what());
    success = false;
  }

  return success;
}

bool ResultExporter::ExportQrDetectionResult(const QRDetectionResult& qr_result, const std::string& scene_name) {
  if (!qr_result.success || !config_.save_intermediate_results) {
    return true;  // 不是错误，只是不需要保存
  }

  bool success = true;
  std::string base_name = "qr_detection";
  if (!scene_name.empty()) {
    base_name += "_" + scene_name;
  }

  try {
    // 保存处理后的图像
    if (!qr_result.processed_image.empty()) {
      ExportImage(qr_result.processed_image, base_name + "_result", "png");
    }

    // 保存检测到的3D圆心
    if (qr_result.circle_centers && !qr_result.circle_centers->empty()) {
      ExportPointCloud(qr_result.circle_centers, base_name + "_centers", true);
    }

    if (verbose_) {
      std::cout << "[ResultExporter] Exported QR detection results for: " << scene_name << std::endl;
    }
  } catch (const std::exception& e) {
    last_error_ = "Failed to export QR detection result: " + std::string(e.what());
    success = false;
  }

  return success;
}

bool ResultExporter::ExportCalibrationReport(const std::vector<CalibrationResultPackage>& results,
                                             const std::string& report_name) {
  if (results.empty()) {
    return false;
  }

  // 导出CSV格式的统计报告
  std::string csv_path = GenerateOutputPath(report_name, OutputFormat::kCsv);

  return ExportAsCsv(results, csv_path);
}

bool ResultExporter::ExportPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename,
                                      bool binary) {
  if (!cloud || cloud->empty()) {
    return false;
  }

  try {
    std::string filepath = GetActualOutputDirectory() + "/" + filename + ".pcd";

    int result;
    if (binary) {
      result = pcl::io::savePCDFileBinary(filepath, *cloud);
    } else {
      result = pcl::io::savePCDFileASCII(filepath, *cloud);
    }

    if (result == 0) {
      if (verbose_) {
        std::cout << "[ResultExporter] Saved point cloud (" << cloud->size() << " points) to: " << filepath
                  << std::endl;
      }
      return true;
    } else {
      last_error_ = "Failed to save point cloud to: " + filepath;
      return false;
    }
  } catch (const std::exception& e) {
    last_error_ = "Error saving point cloud: " + std::string(e.what());
    return false;
  }
}

bool ResultExporter::ExportColoredPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                             const std::string& filename, bool binary) {
  if (!cloud || cloud->empty()) {
    return false;
  }

  try {
    std::string filepath = GetActualOutputDirectory() + "/" + filename + ".pcd";

    int result;
    if (binary) {
      result = pcl::io::savePCDFileBinary(filepath, *cloud);
    } else {
      result = pcl::io::savePCDFileASCII(filepath, *cloud);
    }

    if (result == 0) {
      if (verbose_) {
        std::cout << "[ResultExporter] Saved colored point cloud (" << cloud->size() << " points) to: " << filepath
                  << std::endl;
      }
      return true;
    } else {
      last_error_ = "Failed to save colored point cloud to: " + filepath;
      return false;
    }
  } catch (const std::exception& e) {
    last_error_ = "Error saving colored point cloud: " + std::string(e.what());
    return false;
  }
}

bool ResultExporter::ExportImage(const cv::Mat& image, const std::string& filename, const std::string& format,
                                 int quality) {
  if (image.empty()) {
    return false;
  }

  try {
    std::string filepath = GetActualOutputDirectory() + "/" + filename + "." + format;

    std::vector<int> compression_params;
    if (format == "jpg" || format == "jpeg") {
      compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params.push_back(quality);
    } else if (format == "png") {
      compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);  // 最大压缩
    }

    bool result = cv::imwrite(filepath, image, compression_params);

    if (result) {
      if (verbose_) {
        std::cout << "[ResultExporter] Saved image (" << image.cols << "x" << image.rows << ") to: " << filepath
                  << std::endl;
      }
      return true;
    } else {
      last_error_ = "Failed to save image to: " + filepath;
      return false;
    }
  } catch (const std::exception& e) {
    last_error_ = "Error saving image: " + std::string(e.what());
    return false;
  }
}

void ResultExporter::SetExportConfig(const ExportConfig& config) {
  config_ = config;
  EnsureOutputDirectoryExists();

  // 重新初始化格式计数器
  format_counts_.clear();
  for (auto format : config_.enabled_formats) {
    format_counts_[format] = 0;
  }
}

ExportConfig ResultExporter::GetExportConfig() const { return config_; }

bool ResultExporter::CreateDirectory(const std::string& path) {
  if (path.empty()) return true;

  size_t pos = 0;
  std::string full_path = path;
  // 确保统一路径分隔符
  std::replace(full_path.begin(), full_path.end(), '\\', '/');

  while ((pos = full_path.find('/', pos + 1)) != std::string::npos) {
    std::string subdir = full_path.substr(0, pos);
    struct stat st;
    if (stat(subdir.c_str(), &st) != 0) {
#ifdef _WIN32
      if (_mkdir(subdir.c_str()) != 0 && errno != EEXIST) return false;
#else
      if (mkdir(subdir.c_str(), 0755) != 0 && errno != EEXIST) return false;
#endif
    }
  }

  // 创建最后一级
  struct stat st;
  if (stat(full_path.c_str(), &st) != 0) {
#ifdef _WIN32
    if (_mkdir(full_path.c_str()) != 0 && errno != EEXIST) return false;
#else
    if (mkdir(full_path.c_str(), 0755) != 0 && errno != EEXIST) return false;
#endif
  }
  return true;
}

std::string ResultExporter::GenerateTimestampedFilename(const std::string& base_name,
                                                        const std::string& extension) const {
  if (!config_.include_timestamp) {
    return base_name + extension;
  }

  std::string timestamp = GetCurrentTimestamp();
  return base_name + "_" + timestamp + extension;
}

bool ResultExporter::ValidateOutputPath(const std::string& filepath) const {
  // 简单验证：检查目录是否可写
  std::string directory = filepath.substr(0, filepath.find_last_of('/'));
  return CreateDirectory(directory);
}

void ResultExporter::SetVerbose(bool verbose) { verbose_ = verbose; }

std::string ResultExporter::GetLastError() const { return last_error_; }

std::string ResultExporter::GetExportStats() const {
  std::stringstream ss;
  ss << "Export Statistics:\n";
  ss << "  Total exports: " << total_exports_ << "\n";
  ss << "  Successful exports: " << successful_exports_ << "\n";
  ss << "  Success rate: " << (total_exports_ > 0 ? (100.0 * successful_exports_ / total_exports_) : 0.0) << "%\n";
  ss << "  Format breakdown:\n";
  for (const auto& pair : format_counts_) {
    ss << "    " << GetFormatExtension(pair.first) << ": " << pair.second << " files\n";
  }
  return ss.str();
}

// 私有方法实现

bool ResultExporter::ExportAsTxt(const CalibrationResultPackage& result_package, const Params& camera_params,
                                 const std::string& filepath) const {
  try {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    const auto& result = result_package.transformation_result;

    // 写入标题和元信息
    file << "# Lidar-Camera-Calib Calibration Result\n";
    file << "# Generated at: " << GetCurrentTimestamp() << "\n";
    if (!result_package.scene_name.empty()) {
      file << "# Scene: " << result_package.scene_name << "\n";
    }
    file << "# RMSE: " << std::fixed << std::setprecision(6) << result.rmse << " m\n";
    file << "# Points used: " << result.num_points << "\n";
    file << "\n";

    // // 相机参数
    // file << "# Camera parameters\n";
    // file << "cam_model: Pinhole\n";
    // file << "cam_fx: " << camera_params.fx << "\n";
    // file << "cam_fy: " << camera_params.fy << "\n";
    // file << "cam_cx: " << camera_params.cx << "\n";
    // file << "cam_cy: " << camera_params.cy << "\n";
    // if (camera_params.distortion_coeffs.size() >= 4) {
    //   file << "cam_d0: " << camera_params.distortion_coeffs[0] << "\n";  // k1
    //   file << "cam_d1: " << camera_params.distortion_coeffs[1] << "\n";  // k2
    //   file << "cam_d2: " << camera_params.distortion_coeffs[2] << "\n";  // p1 or k3
    //   file << "cam_d3: " << camera_params.distortion_coeffs[3] << "\n";  // p2 or k4
    // }
    // file << "\n";

    // 变换矩阵
    file << "# Transformation matrix T_cam_lidar\n";
    file << "Rcl: [";
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        file << std::setw(10) << std::fixed << std::setprecision(6) << result.rotation(i, j);
        if (i < 2 || j < 2) file << ",";
        if (j == 2 && i < 2) file << "\n      ";
      }
    }
    file << "]\n";

    file << "Pcl: [";
    for (int i = 0; i < 3; ++i) {
      file << std::setw(10) << std::fixed << std::setprecision(6) << result.translation(i);
      if (i < 2) file << ", ";
    }
    file << "]\n";

    // 额外指标
    if (!result_package.additional_metrics.empty()) {
      file << "\n# Additional metrics\n";
      for (const auto& metric : result_package.additional_metrics) {
        file << metric.first << ": " << metric.second << "\n";
      }
    }

    file.close();
    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export TXT format: " + std::string(e.what());
    return false;
  }
}

bool ResultExporter::ExportAsYaml(const CalibrationResultPackage& result_package, const Params& camera_params,
                                  const std::string& filepath) const {
  try {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    const auto& result = result_package.transformation_result;

    // 提取相机ID数字部分
    std::string numeric_id;
    for (char c : camera_params.camera_id) {
      if (std::isdigit(c)) {
        numeric_id += c;
      }
    }
    if (numeric_id.empty()) {
      numeric_id = camera_params.camera_id;
    }

    file << "lidar_to_camera_" << numeric_id << ":\n";
    file << "  calibrated: true\n";
    file << "  \"transform:\": [ ";

    file << std::fixed << std::setprecision(4);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (i == 0 && j == 0) {
          // 第一个元素不需要前置逗号
        } else {
          file << ", ";
          if (j == 0) {
            file << std::endl << "                ";
          }
        }
        file << std::setw(8) << result.transformation(i, j);
      }
    }
    file << "]\n";

    file.close();
    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export YAML format: " + std::string(e.what());
    return false;
  }
}

bool ResultExporter::ExportAsJson(const CalibrationResultPackage& result_package, const Params& camera_params,
                                  const std::string& filepath) const {
  try {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    const auto& result = result_package.transformation_result;

    file << "{\n";
    file << "  \"calibration_info\": {\n";
    file << "    \"timestamp\": \"" << GetCurrentTimestamp() << "\",\n";
    if (!result_package.scene_name.empty()) {
      file << "    \"scene_name\": \"" << result_package.scene_name << "\",\n";
    }
    file << "    \"rmse\": " << result.rmse << ",\n";
    file << "    \"points_used\": " << result.num_points << "\n";
    file << "  },\n";

    file << "  \"camera_parameters\": {\n";
    file << "    \"model\": \"Pinhole\",\n";
    file << "    \"fx\": " << camera_params.fx << ",\n";
    file << "    \"fy\": " << camera_params.fy << ",\n";
    file << "    \"cx\": " << camera_params.cx << ",\n";
    file << "    \"cy\": " << camera_params.cy << ",\n";
    if (camera_params.distortion_coeffs.size() >= 4) {
      file << "cam_d0: " << camera_params.distortion_coeffs[0] << "\n";  // k1
      file << "cam_d1: " << camera_params.distortion_coeffs[1] << "\n";  // k2
      file << "cam_d2: " << camera_params.distortion_coeffs[2] << "\n";  // p1 or k3
      file << "cam_d3: " << camera_params.distortion_coeffs[3] << "\n";  // p2 or k4
    }
    file << "  },\n";

    file << "  \"transformation\": {\n";
    file << "    \"rotation_matrix\": [\n";
    for (int i = 0; i < 3; ++i) {
      file << "      [";
      for (int j = 0; j < 3; ++j) {
        file << std::fixed << std::setprecision(6) << result.rotation(i, j);
        if (j < 2) file << ", ";
      }
      file << "]";
      if (i < 2) file << ",";
      file << "\n";
    }
    file << "    ],\n";

    file << "    \"translation_vector\": [";
    for (int i = 0; i < 3; ++i) {
      file << std::fixed << std::setprecision(6) << result.translation(i);
      if (i < 2) file << ", ";
    }
    file << "],\n";

    file << "    \"transformation_matrix\": [\n";
    for (int i = 0; i < 4; ++i) {
      file << "      [";
      for (int j = 0; j < 4; ++j) {
        file << std::fixed << std::setprecision(6) << result.transformation(i, j);
        if (j < 3) file << ", ";
      }
      file << "]";
      if (i < 3) file << ",";
      file << "\n";
    }
    file << "    ]\n";
    file << "  }";

    if (!result_package.additional_metrics.empty()) {
      file << ",\n  \"additional_metrics\": {\n";
      size_t count = 0;
      for (const auto& metric : result_package.additional_metrics) {
        file << "    \"" << metric.first << "\": " << metric.second;
        if (++count < result_package.additional_metrics.size()) file << ",";
        file << "\n";
      }
      file << "  }";
    }

    file << "\n}\n";

    file.close();
    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export JSON format: " + std::string(e.what());
    return false;
  }
}

bool ResultExporter::ExportAsXml(const CalibrationResultPackage& result_package, const Params& camera_params,
                                 const std::string& filepath) const {
  try {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    const auto& result = result_package.transformation_result;

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<calibration_result>\n";

    file << "  <calibration_info>\n";
    file << "    <timestamp>" << GetCurrentTimestamp() << "</timestamp>\n";
    if (!result_package.scene_name.empty()) {
      file << "    <scene_name>" << result_package.scene_name << "</scene_name>\n";
    }
    file << "    <rmse>" << result.rmse << "</rmse>\n";
    file << "    <points_used>" << result.num_points << "</points_used>\n";
    file << "  </calibration_info>\n\n";

    file << "  <camera_parameters>\n";
    file << "    <model>Pinhole</model>\n";
    file << "    <fx>" << camera_params.fx << "</fx>\n";
    file << "    <fy>" << camera_params.fy << "</fy>\n";
    file << "    <cx>" << camera_params.cx << "</cx>\n";
    file << "    <cy>" << camera_params.cy << "</cy>\n";
    if (camera_params.distortion_coeffs.size() >= 4) {
      file << "cam_d0: " << camera_params.distortion_coeffs[0] << "\n";  // k1
      file << "cam_d1: " << camera_params.distortion_coeffs[1] << "\n";  // k2
      file << "cam_d2: " << camera_params.distortion_coeffs[2] << "\n";  // p1 or k3
      file << "cam_d3: " << camera_params.distortion_coeffs[3] << "\n";  // p2 or k4
    }
    file << "  </camera_parameters>\n\n";

    file << "  <transformation>\n";
    file << "    <rotation_matrix>\n";
    for (int i = 0; i < 3; ++i) {
      file << "      <row" << i << ">";
      for (int j = 0; j < 3; ++j) {
        file << std::fixed << std::setprecision(6) << result.rotation(i, j);
        if (j < 2) file << " ";
      }
      file << "</row" << i << ">\n";
    }
    file << "    </rotation_matrix>\n";

    file << "    <translation_vector>";
    for (int i = 0; i < 3; ++i) {
      file << std::fixed << std::setprecision(6) << result.translation(i);
      if (i < 2) file << " ";
    }
    file << "</translation_vector>\n";
    file << "  </transformation>\n";

    if (!result_package.additional_metrics.empty()) {
      file << "\n  <additional_metrics>\n";
      for (const auto& metric : result_package.additional_metrics) {
        file << "    <" << metric.first << ">" << metric.second << "</" << metric.first << ">\n";
      }
      file << "  </additional_metrics>\n";
    }

    file << "</calibration_result>\n";

    file.close();
    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export XML format: " + std::string(e.what());
    return false;
  }
}

bool ResultExporter::ExportAsCsv(const std::vector<CalibrationResultPackage>& results,
                                 const std::string& filepath) const {
  try {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    // CSV 头部
    file << "Scene,Timestamp,RMSE,Points_Used,Success,Translation_X,Translation_Y,Translation_Z,";
    file << "Rotation_00,Rotation_01,Rotation_02,Rotation_10,Rotation_11,Rotation_12,Rotation_20,Rotation_21,Rotation_"
            "22\n";

    // 数据行
    for (const auto& result_package : results) {
      const auto& result = result_package.transformation_result;

      file << "\"" << result_package.scene_name << "\",";
      file << "\"" << result_package.timestamp << "\",";
      file << result.rmse << ",";
      file << result.num_points << ",";
      file << (result.success ? "true" : "false") << ",";

      // 平移向量
      file << std::fixed << std::setprecision(6) << result.translation(0) << ",";
      file << std::fixed << std::setprecision(6) << result.translation(1) << ",";
      file << std::fixed << std::setprecision(6) << result.translation(2) << ",";

      // 旋转矩阵
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          file << std::fixed << std::setprecision(6) << result.rotation(i, j);
          if (i < 2 || j < 2) file << ",";
        }
      }
      file << "\n";
    }

    file.close();
    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export CSV format: " + std::string(e.what());
    return false;
  }
}

std::string ResultExporter::FormatMatrix(const Eigen::Matrix4d& matrix, const std::string& format) const {
  std::stringstream ss;

  if (format == "yaml") {
    for (int i = 0; i < 4; ++i) {
      ss << "  - [";
      for (int j = 0; j < 4; ++j) {
        ss << std::fixed << std::setprecision(6) << matrix(i, j);
        if (j < 3) ss << ", ";
      }
      ss << "]";
      if (i < 3) ss << "\n";
    }
  } else {
    // txt format
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        ss << std::setw(10) << std::fixed << std::setprecision(6) << matrix(i, j);
        if (j < 3) ss << " ";
      }
      if (i < 3) ss << "\n";
    }
  }

  return ss.str();
}

std::string ResultExporter::FormatVector(const Eigen::Vector3d& vector, const std::string& format) const {
  std::stringstream ss;

  if (format == "yaml") {
    ss << "[";
    for (int i = 0; i < 3; ++i) {
      ss << std::fixed << std::setprecision(6) << vector(i);
      if (i < 2) ss << ", ";
    }
    ss << "]";
  } else {
    // txt format
    for (int i = 0; i < 3; ++i) {
      ss << std::setw(10) << std::fixed << std::setprecision(6) << vector(i);
      if (i < 2) ss << " ";
    }
  }

  return ss.str();
}

bool ResultExporter::ExportMultiSceneResultOriginalFormat(
    const std::vector<CalibrationResultPackage>& individual_results,
    const CalibrationResultPackage& final_result) const {
  try {
    // 创建multi_calib_result.txt文件（与原始ROS版本格式完全匹配）
    std::string filepath = GetActualOutputDirectory() + "/multi_calib_result.txt";
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      last_error_ = "Failed to create multi-scene result file: " + filepath;
      return false;
    }

    const auto& result = final_result.transformation_result;

    // 输出格式
    fout << "# Using " << individual_results.size() << " datasets for calibration\n";
    fout << "# Selected indices: ";
    for (size_t i = 0; i < individual_results.size(); ++i) {
      if (i > 0) fout << ",";
      fout << i;  // 这里使用顺序索引，实际应用中可能需要传入真实的索引
    }
    fout << "\n";
    fout << std::fixed << std::setprecision(6);

    // 旋转矩阵
    fout << "Rcl: [ " << std::setw(9) << result.rotation(0, 0) << ", " << std::setw(9) << result.rotation(0, 1) << ", "
         << std::setw(9) << result.rotation(0, 2) << ",\n"
         << "      " << std::setw(9) << result.rotation(1, 0) << ", " << std::setw(9) << result.rotation(1, 1) << ", "
         << std::setw(9) << result.rotation(1, 2) << ",\n"
         << "      " << std::setw(9) << result.rotation(2, 0) << ", " << std::setw(9) << result.rotation(2, 1) << ", "
         << std::setw(9) << result.rotation(2, 2) << "]\n";

    // 平移向量
    fout << "Pcl: [ " << std::setw(9) << result.translation(0) << ", " << std::setw(9) << result.translation(1) << ", "
         << std::setw(9) << result.translation(2) << "]\n";

    // RMSE
    fout << "RMSE: " << result.rmse << "\n";

    fout.close();

    if (verbose_) {
      std::cout << "[ResultExporter] Multi-scene calibration results saved to " << filepath << std::endl;
    }

    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export multi-scene result in original format: " + std::string(e.what());
    return false;
  }
}

std::string ResultExporter::GetCurrentTimestamp() const {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}

bool ResultExporter::EnsureOutputDirectoryExists() const { return CreateDirectory(config_.output_directory); }

std::string ResultExporter::GetFormatExtension(OutputFormat format) const {
  auto it = kFormatExtensions.find(format);
  return it != kFormatExtensions.end() ? it->second : ".txt";
}

std::string ResultExporter::GenerateOutputPath(const std::string& filename, OutputFormat format) const {
  std::string extension = GetFormatExtension(format);
  std::string full_filename = GenerateTimestampedFilename(filename, extension);
  return GetActualOutputDirectory() + "/" + full_filename;
}

std::string ResultExporter::ExtractDatasetName(const Params& params) const {
  std::string dataset_name = "unknown_dataset";

  // 优先从dataset_dir提取
  if (!params.dataset_dir.empty()) {
    size_t last_slash = params.dataset_dir.find_last_of("/\\");
    if (last_slash != std::string::npos) {
      dataset_name = params.dataset_dir.substr(last_slash + 1);
    } else {
      dataset_name = params.dataset_dir;
    }
  } else if (!params.image_path.empty()) {
    // 从image_path推导数据集名称
    size_t last_slash = params.image_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
      std::string dir_path = params.image_path.substr(0, last_slash);
      size_t prev_slash = dir_path.find_last_of("/\\");
      if (prev_slash != std::string::npos) {
        dataset_name = dir_path.substr(prev_slash + 1);
      }
    }
  }

  // 移除可能的路径相对前缀
  if (dataset_name.substr(0, 2) == "./") {
    dataset_name = dataset_name.substr(2);
  }

  return dataset_name;
}

std::string ResultExporter::GetActualOutputDirectory() const {
  if (config_.dataset_name.empty()) {
    return config_.output_directory;
  } else {
    return config_.output_directory + "/" + config_.dataset_name;
  }
}

bool ResultExporter::ExportLidarToCameraYaml(const RigidTransformResult& result, const Params& params) {
  try {
    // 提取数字部分
    std::string numeric_id = "";
    for (char c : params.camera_id) {
      if (std::isdigit(c)) {
        numeric_id += c;
      }
    }
    if (numeric_id.empty()) {
      numeric_id = params.camera_id;
    }

    std::string filename = "lidar2camera_" + numeric_id + ".yaml";
    std::string filepath = params.output_path + "/" + filename;

    // 确保输出目录存在
    CreateDirectory(params.output_path);

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      last_error_ = "Failed to create specialized YAML file: " + filepath;
      return false;
    }

    fout << "lidar_to_camera_" << numeric_id << ":" << std::endl;
    fout << "  calibrated: true" << std::endl;
    fout << "  \"transform:\": [ ";

    fout << std::fixed << std::setprecision(4);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (i == 0 && j == 0) {
          // 第一个元素不需要前置逗号
        } else {
          fout << ", ";
          if (j == 0) {
            fout << std::endl << "                ";
          }
        }
        fout << std::setw(8) << result.transformation(i, j);
      }
    }
    fout << "]" << std::endl;

    fout.close();
    return true;
  } catch (const std::exception& e) {
    last_error_ = "Failed to export specialized YAML: " + std::string(e.what());
    return false;
  }
}