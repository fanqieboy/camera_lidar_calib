/**
 * @file visualizer.h
 * @brief 可视化模块 - 移除ROS依赖的独立可视化系统
 *
 * 提供独立的3D点云和图像可视化功能，替代ROS RViz
 * 支持标定结果验证、中间处理结果显示、多场景对比等功能
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

// 前向声明
struct Params;
struct LidarProcessingResult;
struct QRDetectionResult;
struct RigidTransformResult;

/**
 * @brief 可视化配置结构体
 *
 * 包含点云文件保存和图像显示相关的配置参数
 */
struct VisualizationConfig {
  bool save_point_clouds;         ///< 是否保存点云文件
  bool save_intermediate_results; ///< 是否保存中间处理结果
  bool enable_colored_pointcloud; ///< 是否启用点云着色
  std::string output_directory;   ///< 输出目录路径
  std::string dataset_name;       ///< 数据集名称（用于创建子目录）
  std::string file_prefix;        ///< 文件名前缀
  std::string default_color;      ///< 默认点云颜色（格式: "r,g,b"）
  bool save_as_binary;           ///< 是否以二进制格式保存PCD文件

  VisualizationConfig()
      : save_point_clouds(true),
        save_intermediate_results(true),
        enable_colored_pointcloud(true),
        output_directory("./output"),
        dataset_name(""),
        file_prefix(""),
        default_color("128,128,128"),
        save_as_binary(false) {}
};

/**
 * @class Visualizer
 * @brief 独立的点云文件保存和图像显示模块
 *
 * 该类负责提供以下核心功能：
 * - 点云数据保存为PCD文件
 * - 标定结果文件输出
 * - 中间处理结果文件保存
 * - 图像显示和保存
 * - 多场景标定结果管理
 * - 文件输出配置管理
 *
 * @note 线程安全性：不是线程安全的，需要在单一线程中使用
 * @see CalibCore, LidarDetector, QrDetector
 */
class Visualizer {
 public:
  /**
   * @brief 构造函数
   *
   * @param params [in] 标定参数，包含相机内参和畸变系数
   * @note 会自动从参数中提取相机参数并初始化可视化器
   */
  explicit Visualizer(const Params& params);

  /**
   * @brief 析构函数
   *
   * 自动清理可视化资源，关闭窗口，停止运行循环
   */
  ~Visualizer();

  /**
   * @brief 显示LiDAR处理结果
   *
   * @param result [in] LiDAR处理结果，包含各阶段点云数据
   * @param show_intermediate [in] 是否显示中间处理结果（默认true）
   * @note 会创建多视口布局显示不同阶段的点云处理结果
   */
  void ShowLidarProcessingResult(const LidarProcessingResult& result, bool show_intermediate = true);

  /**
   * @brief 显示QR检测结果
   *
   * @param result [in] QR检测结果，包含检测到的标记和3D坐标
   * @param show_processed_image [in] 是否显示处理后的图像（默认true）
   * @note 会同时显示2D检测图像和3D圆心标记
   */
  void ShowQrDetectionResult(const QRDetectionResult& result, bool show_processed_image = true);

  /**
   * @brief 显示标定结果验证
   *
   * @param calib_result [in] 标定结果，包含变换矩阵和精度信息
   * @param original_cloud [in] 原始点云数据
   * @param camera_image [in] 相机图像
   * @param camera_matrix [in] 相机内参矩阵
   * @param distortion_coeffs [in] 畸变系数
   * @note 会创建双视口对比显示原始点云和标定后的着色点云
   */
  void ShowCalibrationResult(const RigidTransformResult& calib_result,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& original_cloud, const cv::Mat& camera_image,
                             const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs);

  /**
   * @brief 显示多场景标定对比
   *
   * @param scene_results [in] 多个场景的标定结果
   * @param final_result [in] 最终联合标定结果
   * @note 显示各场景的RMSE对比和最终融合结果
   */
  void ShowMultiSceneComparison(const std::vector<RigidTransformResult>& scene_results,
                                const RigidTransformResult& final_result);
  /**
   * @brief 显示失败的LiDAR处理中间结果（诊断用）
   *
   * @param result [in] 失败的LiDAR处理结果，包含中间处理步骤
   * @param original_cloud [in] 原始点云数据
   * @note 即使处理失败，也会显示所有可用的中间处理结果用于诊断
   */
  void ShowFailedLidarProcessing(const LidarProcessingResult& result,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& original_cloud);

  /**
   * @brief 显示失败的QR检测结果（诊断用）
   *
   * @param result [in] 失败的QR检测结果，包含部分检测信息
   * @param original_image [in] 原始相机图像
   * @note 显示原始图像和任何部分检测到的标记，帮助诊断QR检测失败原因
   */
  void ShowFailedQrDetection(const QRDetectionResult& result, const cv::Mat& original_image);

  /**
   * @brief 显示标定失败时的输入数据（诊断用）
   *
   * @param qr_result [in] QR检测结果
   * @param lidar_result [in] LiDAR检测结果
   * @param camera_image [in] 相机图像
   * @param point_cloud [in] 点云数据
   * @param camera_matrix [in] 相机内参矩阵
   * @param distortion_coeffs [in] 畸变系数
   * @note 显示标定输入数据，帮助分析标定失败的原因
   */
  void ShowFailedCalibration(const QRDetectionResult& qr_result, const LidarProcessingResult& lidar_result,
                            const cv::Mat& camera_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud,
                            const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs);

  /**
   * @brief 保存点云为PCD文件
   *
   * @param cloud [in] 点云数据
   * @param filename [in] 文件名（不含路径）
   * @return bool 保存是否成功
   */
  bool SavePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename);

  /**
   * @brief 保存彩色点云为PCD文件
   *
   * @param cloud [in] 彩色点云数据
   * @param filename [in] 文件名（不含路径）
   * @return bool 保存是否成功
   */
  bool SaveColoredPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename);

  /**
   * @brief 显示图像
   *
   * @param image [in] 图像数据
   * @param window_name [in] 窗口名称
   * @param wait_key [in] 是否等待按键（默认false）
   */
  void ShowImage(const cv::Mat& image, const std::string& window_name, bool wait_key = false);

  /**
   * @brief 保存圆心标记为PCD文件
   *
   * @param centers [in] 圆心点云
   * @param filename [in] 文件名
   * @param color [in] 颜色（r,g,b范围0-1，默认红色）
   * @return bool 保存是否成功
   */
  bool SaveCenterMarkers(const pcl::PointCloud<pcl::PointXYZI>::Ptr& centers, const std::string& filename,
                        const std::vector<double>& color = {1.0, 0.0, 0.0});

  /**
   * @brief 创建输出目录结构
   *
   * @return bool 创建是否成功
   */
  bool CreateOutputDirectories();

  /**
   * @brief 清除输出目录中的旧文件
   */
  void ClearOutputDirectory();

  /**
   * @brief 设置可视化配置
   *
   * @param config [in] 可视化配置
   */
  void SetVisualizationConfig(const VisualizationConfig& config);

  /**
   * @brief 获取当前可视化配置
   *
   * @return VisualizationConfig 当前配置
   */
  VisualizationConfig GetVisualizationConfig() const;


  /**
   * @brief 启用/禁用详细输出
   *
   * @param verbose [in] 是否启用详细输出
   */
  void SetVerbose(bool verbose);

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息
   */
  std::string GetLastError() const;

 private:
  /**
   * @brief 初始化输出目录
   */
  void InitializeOutputDirectory();

  /**
   * @brief 应用文件保存配置
   */
  void ApplyOutputConfig();

  /**
   * @brief 解析颜色字符串
   *
   * @param color_str [in] 颜色字符串格式为"r,g,b"
   * @return std::vector<double> RGB颜色值
   */
  std::vector<double> ParseColorString(const std::string& color_str) const;

  /**
   * @brief 构建完整的文件路径
   *
   * @param filename [in] 文件名
   * @param subdirectory [in] 子目录名（可选）
   * @return std::string 完整的文件路径
   */
  std::string BuildFilePath(const std::string& filename, const std::string& subdirectory = "") const;

  /**
   * @brief 获取实际输出目录（包含数据集子目录）
   *
   * @return std::string 实际输出目录路径
   */
  std::string GetActualOutputDirectory() const;

  /**
   * @brief 创建着色点云用于标定结果验证
   *
   * @param cloud [in] 原始点云
   * @param transformation [in] 变换矩阵
   * @param camera_image [in] 相机图像
   * @param camera_matrix [in] 相机内参
   * @param distortion_coeffs [in] 畸变系数
   * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 着色点云
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CreateColoredPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                                                 const Eigen::Matrix4d& transformation,
                                                                 const cv::Mat& camera_image,
                                                                 const cv::Mat& camera_matrix,
                                                                 const cv::Mat& distortion_coeffs) const;

  /**
   * @brief 为点云设置颜色
   *
   * @param cloud [in] 输入点云
   * @param colored_cloud [out] 输出彩色点云
   * @param color [in] RGB颜色值
   */
  void SetPointCloudColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
                          const std::vector<double>& color) const;

  /**
   * @brief 创建虚拟参考点云确保可视化器稳定性
   *
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr 包含少量参考点的虚拟点云
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr CreateDummyPointCloud() const;

 private:
  VisualizationConfig config_;                     ///< 文件保存配置参数

  // 标定参数相关
  cv::Mat camera_matrix_;      ///< 相机内参矩阵
  cv::Mat distortion_coeffs_;  ///< 相机畸变系数

  // 状态管理
  bool is_initialized_;             ///< 是否已初始化标志
  mutable std::string last_error_;  ///< 最后发生的错误信息
  bool verbose_;                    ///< 详细输出模式标志

  // 文件保存管理
  int scene_counter_;               ///< 场景计数器（多场景模式用）
  int file_counter_;                ///< 文件计数器

  // 静态常量
  static const std::vector<std::vector<double>> kDefaultColors;  ///< 默认颜色集合
};

using VisualizerPtr = std::shared_ptr<Visualizer>;