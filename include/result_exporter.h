/**
 * @file result_exporter.h
 * @brief 结果导出模块 - 多格式标定结果导出
 *
 * 负责将标定结果导出为多种标准格式，包括TXT、YAML、JSON、XML等格式，
 * 并支持点云和图像文件的导出。
 */

#pragma once

#include <ctime>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "calib_core.h"  // 提供 RigidTransformResult 的完整定义

// 前置声明
struct Params;
struct LidarProcessingResult;
struct QRDetectionResult;

/**
 * @brief 输出格式枚举
 *
 * 定义支持的输出格式类型
 */
enum class OutputFormat {
  kTxt,   ///< 纯文本格式
  kYaml,  ///< YAML格式
  kJson,  ///< JSON格式
  kXml,   ///< XML格式
  kCsv    ///< CSV格式（用于统计数据）
};

/**
 * @brief 输出配置结构
 *
 * 定义结果导出的各种配置选项
 */
struct ExportConfig {
  std::string output_directory;               ///< 输出目录
  std::string filename_prefix;                ///< 文件名前缀
  std::string dataset_name;                   ///< 数据集名称（用于创建子目录）
  bool include_timestamp;                     ///< 是否包含时间戳
  bool save_intermediate_results;             ///< 是否保存中间结果
  bool save_point_clouds;                     ///< 是否保存点云
  bool save_images;                           ///< 是否保存图像
  bool compress_point_clouds;                 ///< 是否压缩点云文件
  std::vector<OutputFormat> enabled_formats;  ///< 启用的输出格式

  ExportConfig()
      : output_directory("./output"),
        filename_prefix("calib_result"),
        dataset_name(""),
        include_timestamp(true),
        save_intermediate_results(false),
        save_point_clouds(true),
        save_images(true),
        compress_point_clouds(false) {
    enabled_formats = {OutputFormat::kTxt, OutputFormat::kYaml};
  }
};

/**
 * @brief 标定结果包装结构
 *
 * 包含完整的标定结果数据，包括变换矩阵、点云、图像等信息
 */
struct CalibrationResultPackage {
  RigidTransformResult transformation_result;            ///< 变换结果
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_centers;    ///< LiDAR圆心
  pcl::PointCloud<pcl::PointXYZI>::Ptr camera_centers;   ///< 相机圆心
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;  ///< 着色点云
  cv::Mat camera_image;                                  ///< 相机图像
  cv::Mat processed_image;                               ///< 处理后的图像
  std::string scene_name;                                ///< 场景名称
  std::string timestamp;                                 ///< 时间戳
  int camera_direction;                                  ///< 相机方向参数 (0:正前方,1:右侧,2:背后,3:左侧)
  std::map<std::string, double> additional_metrics;      ///< 额外指标

  CalibrationResultPackage() {
    lidar_centers = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    camera_centers = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    camera_direction = 0;  // 默认为正前方
  }
};

/**
 * @class ResultExporter
 * @brief 结果导出器类 - 多格式标定结果导出
 *
 * 该类负责将标定结果导出为多种标准格式，包括：
 * - TXT格式 - 兼容LIDAR-CAMERA-CALIB-FORMAT格式
 * - YAML格式 - 结构化配置文件格式
 * - JSON格式 - Web友好格式
 * - XML格式 - 标准结构化格式
 * - 点云文件 - PCD格式（可选压缩）
 * - 图像文件 - PNG/JPG格式
 *
 * @note 该类是线程安全的，可在多线程环境中使用
 * @see CalibrationResultPackage, ExportConfig
 */
class ResultExporter {
 public:
  /**
   * @brief 构造函数
   *
   * @param config 输出配置
   */
  explicit ResultExporter(const ExportConfig& config = ExportConfig());

  /**
   * @brief 析构函数
   */
  ~ResultExporter();

  /**
   * @brief 导出单场景标定结果
   *
   * @param result_package 标定结果包
   * @param camera_params 相机参数
   * @return bool 导出是否成功
   */
  bool ExportSingleSceneResult(const CalibrationResultPackage& result_package, const Params& camera_params);

  /**
   * @brief 导出多场景标定结果
   *
   * @param individual_results 各场景结果列表
   * @param final_result 最终联合标定结果
   * @param camera_params 相机参数
   * @return bool 导出是否成功
   */
  bool ExportMultiSceneResult(const std::vector<CalibrationResultPackage>& individual_results,
                              const CalibrationResultPackage& final_result, const Params& camera_params);

  /**
   * @brief 导出雷达对相机的外参 (特定 YAML 格式)
   *
   * @param result 变换结果
   * @param params 配置参数 (包含 camera_id 和 output_path)
   * @return bool 导出是否成功
   */
  bool ExportLidarToCameraYaml(const RigidTransformResult& result, const Params& params);

  /**
   * @brief 导出LiDAR处理中间结果
   *
   * @param lidar_result LiDAR处理结果
   * @param scene_name 场景名称
   * @return bool 导出是否成功
   */
  bool ExportLidarProcessingResult(const LidarProcessingResult& lidar_result, const std::string& scene_name = "");

  /**
   * @brief 导出QR检测结果
   *
   * @param qr_result QR检测结果
   * @param scene_name 场景名称
   * @return bool 导出是否成功
   */
  bool ExportQrDetectionResult(const QRDetectionResult& qr_result, const std::string& scene_name = "");

  /**
   * @brief 导出标定统计报告
   *
   * @param results 多个标定结果
   * @param report_name 报告名称
   * @return bool 导出是否成功
   */
  bool ExportCalibrationReport(const std::vector<CalibrationResultPackage>& results,
                               const std::string& report_name = "calibration_report");

  /**
   * @brief 导出点云文件
   *
   * @param cloud 点云数据
   * @param filename 文件名（不含扩展名）
   * @param binary 是否使用二进制格式
   * @return bool 导出是否成功
   */
  bool ExportPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename,
                        bool binary = true);

  /**
   * @brief 导出彩色点云文件
   *
   * @param cloud 彩色点云数据
   * @param filename 文件名（不含扩展名）
   * @param binary 是否使用二进制格式
   * @return bool 导出是否成功
   */
  bool ExportColoredPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename,
                               bool binary = true);

  /**
   * @brief 导出图像文件
   *
   * @param image 图像数据
   * @param filename 文件名（不含扩展名）
   * @param format 图像格式（"png", "jpg", "bmp"）
   * @param quality 图像质量（仅JPEG有效，0-100）
   * @return bool 导出是否成功
   */
  bool ExportImage(const cv::Mat& image, const std::string& filename, const std::string& format = "png",
                   int quality = 95);

  /**
   * @brief 设置输出配置
   *
   * @param config 新的输出配置
   */
  void SetExportConfig(const ExportConfig& config);

  /**
   * @brief 获取当前输出配置
   *
   * @return ExportConfig 当前配置
   */
  ExportConfig GetExportConfig() const;

  /**
   * @brief 创建输出目录
   *
   * @param path 目录路径
   * @return bool 创建是否成功
   */
  static bool CreateDirectory(const std::string& path);

  /**
   * @brief 生成带时间戳的文件名
   *
   * @param base_name 基础文件名
   * @param extension 文件扩展名
   * @return std::string 完整文件名
   */
  std::string GenerateTimestampedFilename(const std::string& base_name, const std::string& extension) const;

  /**
   * @brief 验证输出路径
   *
   * @param filepath 文件路径
   * @return bool 路径是否有效
   */
  bool ValidateOutputPath(const std::string& filepath) const;

  /**
   * @brief 设置详细输出模式
   *
   * @param verbose 是否启用详细输出
   */
  void SetVerbose(bool verbose);

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息
   */
  std::string GetLastError() const;

  /**
   * @brief 获取导出统计信息
   *
   * @return std::string 统计信息
   */
  std::string GetExportStats() const;

 private:
  /**
   * @brief 导出TXT格式结果
   */
  bool ExportAsTxt(const CalibrationResultPackage& result_package, const Params& camera_params,
                   const std::string& filepath) const;

  /**
   * @brief 导出原始格式的多场景结果（与ROS版本格式匹配）
   */
  bool ExportMultiSceneResultOriginalFormat(const std::vector<CalibrationResultPackage>& individual_results,
                                           const CalibrationResultPackage& final_result) const;

  /**
   * @brief 导出YAML格式结果
   */
  bool ExportAsYaml(const CalibrationResultPackage& result_package, const Params& camera_params,
                    const std::string& filepath) const;

  /**
   * @brief 导出JSON格式结果
   */
  bool ExportAsJson(const CalibrationResultPackage& result_package, const Params& camera_params,
                    const std::string& filepath) const;

  /**
   * @brief 导出XML格式结果
   */
  bool ExportAsXml(const CalibrationResultPackage& result_package, const Params& camera_params,
                   const std::string& filepath) const;

  /**
   * @brief 导出CSV格式统计数据
   */
  bool ExportAsCsv(const std::vector<CalibrationResultPackage>& results, const std::string& filepath) const;

  /**
   * @brief 格式化矩阵为字符串
   */
  std::string FormatMatrix(const Eigen::Matrix4d& matrix, const std::string& format = "txt") const;

  /**
   * @brief 格式化向量为字符串
   */
  std::string FormatVector(const Eigen::Vector3d& vector, const std::string& format = "txt") const;

  /**
   * @brief 获取当前时间戳字符串
   */
  std::string GetCurrentTimestamp() const;

  /**
   * @brief 确保输出目录存在
   */
  bool EnsureOutputDirectoryExists() const;

  /**
   * @brief 获取格式对应的文件扩展名
   */
  std::string GetFormatExtension(OutputFormat format) const;

  /**
   * @brief 生成完整的输出文件路径
   */
  std::string GenerateOutputPath(const std::string& filename, OutputFormat format) const;

  /**
   * @brief 从配置参数中提取数据集名称
   */
  std::string ExtractDatasetName(const Params& params) const;

  /**
   * @brief 获取实际输出目录（包含数据集子目录）
   */
  std::string GetActualOutputDirectory() const;

 private:
  ExportConfig config_;             ///< 输出配置
  mutable std::string last_error_;  ///< 最后的错误信息
  bool verbose_;                    ///< 详细输出模式

  // 统计信息
  int total_exports_;                          ///< 总导出次数
  int successful_exports_;                     ///< 成功导出次数
  std::map<OutputFormat, int> format_counts_;  ///< 各格式导出计数

  // 格式映射
  static const std::map<OutputFormat, std::string> kFormatExtensions;  ///< 格式扩展名映射
  static const std::map<std::string, OutputFormat> kExtensionFormats;  ///< 扩展名格式映射
};

using ResultExporterPtr = std::shared_ptr<ResultExporter>;
