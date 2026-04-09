/**
 * @file data_loader.h
 * @brief 数据加载系统头文件 - 替代ROS bag和消息类型
 *
 * 该文件定义了独立的数据加载系统，提供标准文件格式的数据输入接口，
 * 支持图像文件(PNG、JPG、BMP等)和点云文件(PCD)的加载，完全替代原有的ROS bag读取功能。
 * 同时支持批量数据加载和多场景数据管理。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 前置声明，避免直接包含
struct Params;

/**
 * @brief 多场景数据结构
 *
 * 封装单个场景的完整数据，包括图像、点云及其元信息。
 * 提供数据有效性验证和便于管理的统一接口。
 */
struct SceneData {
  std::string scene_name;                            ///< 场景名称
  cv::Mat image;                                     ///< 图像数据
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;  ///< 点云数据
  std::string image_path;                            ///< 图像文件路径
  std::string pointcloud_path;                       ///< 点云文件路径
  bool is_valid;                                     ///< 数据是否有效

  SceneData() : point_cloud(new pcl::PointCloud<pcl::PointXYZI>()), is_valid(false) {}
};

/**
 * @class DataLoader
 * @brief 数据加载器类 - 提供标准文件格式的数据输入接口
 *
 * 主要功能:
 * - 加载图像文件（PNG、JPG、BMP等）和点云文件（PCD）
 * - 替代原有的ROS bag读取功能
 * - 支持批量数据加载和多场景数据管理
 * - 提供文件格式验证和错误处理机制
 * - 支持加载统计和详细日志输出
 *
 * @note 线程安全: 该类不保证线程安全，需要外部同步
 * @see SceneData 场景数据结构体
 */
class DataLoader {
 public:
  /**
   * @brief 构造函数
   *
   * 初始化数据加载器，设置默认参数和统计计数器。
   */
  DataLoader();

  /**
   * @brief 析构函数
   *
   * 清理资源并输出最终统计信息（如果启用详细模式）。
   */
  ~DataLoader();

  /**
   * @brief 加载单个图像文件
   *
   * @param image_path 图像文件路径 (in)
   * @param flags OpenCV imread 标志 (in, 默认 IMREAD_UNCHANGED)
   * @return cv::Mat 加载的图像，如果失败则返回空矩阵
   * @throw 不抛出异常，错误信息通过GetLastError()获取
   * @note 支持的格式: PNG, JPG, JPEG, BMP, TIFF, TIF
   */
  cv::Mat LoadImage(const std::string& image_path, int flags = cv::IMREAD_UNCHANGED);

  /**
   * @brief 加载单个点云文件
   *
   * @param pcd_path PCD 文件路径
   * @return pcl::PointCloud<pcl::PointXYZI>::Ptr 加载的点云，如果失败则返回空指针
   * @throw 不抛出异常，错误信息通过GetLastError()获取
   * @note 当前仅支持PCD格式
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr LoadPointCloud(const std::string& pcd_path);

  /**
   * @brief 从参数配置加载单场景数据
   *
   * @param params 参数配置，包含图像路径等信息
   * @return SceneData 场景数据结构
   * @note 点云文件路径根据图像路径自动推导（同目录，同文件名，.pcd扩展名）
   * @warning 需要确保对应的点云文件存在且命名正确
   */
  SceneData LoadSingleScene(const Params& params);

  /**
   * @brief 加载多场景数据
   *
   * @param scene_configs 场景配置列表 (图像路径, 点云路径)
   * @return std::vector<SceneData> 多场景数据列表
   * @note 只返回成功加载的场景，失败的场景会被跳过
   * @warning 加载失败的场景不会包含在返回结果中
   */
  std::vector<SceneData> LoadMultiSceneData(const std::vector<std::pair<std::string, std::string>>& scene_configs);

  /**
   * @brief 从目录批量加载数据
   *
   * @param data_directory 数据目录路径 (in)
   * @param image_extension 图像文件扩展名 (in, 如 ".png", ".jpg")
   * @param pointcloud_extension 点云文件扩展名 (in, 如 ".pcd")
   * @return std::vector<SceneData> 批量加载的数据
   * @note 根据文件名自动匹配图像和点云文件对
   * @warning 只有同时存在图像和点云文件的场景才会被加载
   */
  std::vector<SceneData> LoadBatchData(const std::string& data_directory, const std::string& image_extension = ".png",
                                       const std::string& pointcloud_extension = ".pcd");

  /**
   * @brief 验证图像文件格式
   *
   * @param image_path 图像文件路径
   * @return true 支持的格式
   * @return false 不支持的格式
   */
  static bool IsValidImageFormat(const std::string& image_path);

  /**
   * @brief 验证点云文件格式
   *
   * @param pcd_path 点云文件路径
   * @return true 支持的格式
   * @return false 不支持的格式
   */
  static bool IsValidPointCloudFormat(const std::string& pcd_path);

  /**
   * @brief 获取支持的图像格式列表
   *
   * @return std::vector<std::string> 支持的扩展名列表
   */
  static std::vector<std::string> GetSupportedImageFormats();

  /**
   * @brief 获取支持的点云格式列表
   *
   * @return std::vector<std::string> 支持的扩展名列表
   */
  static std::vector<std::string> GetSupportedPointCloudFormats();

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息
   */
  std::string GetLastError() const;

  /**
   * @brief 获取加载统计信息
   *
   * @return std::string 统计信息
   */
  std::string GetLoadingStats() const;

  /**
   * @brief 清除错误信息和统计
   */
  void ClearStatus();

  /**
   * @brief 设置详细输出模式
   *
   * @param verbose 是否启用详细输出
   */
  void SetVerbose(bool verbose);

 private:
  /**
   * @brief 验证文件是否存在且可读
   *
   * @param file_path 文件路径
   * @return true 文件有效
   * @return false 文件无效
   */
  bool ValidateFile(const std::string& file_path);

  /**
   * @brief 从文件路径提取文件名（无扩展名）
   *
   * @param file_path 文件路径
   * @return std::string 文件名
   */
  std::string ExtractFileName(const std::string& file_path);

  /**
   * @brief 获取文件扩展名
   *
   * @param file_path 文件路径
   * @return std::string 扩展名（包含点）
   */
  std::string GetFileExtension(const std::string& file_path);

  /**
   * @brief 更新加载统计
   *
   * @param success 是否成功
   * @param file_type 文件类型
   */
  void UpdateStats(bool success, const std::string& file_type);

 private:
  mutable std::string last_error_;  ///< 最后的错误信息
  bool verbose_;                    ///< 是否启用详细输出

  // 加载统计
  int images_loaded_;       ///< 已加载图像数量
  int pointclouds_loaded_;  ///< 已加载点云数量
  int images_failed_;       ///< 图像加载失败数量
  int pointclouds_failed_;  ///< 点云加载失败数量

  // 支持的格式
  static const std::vector<std::string> kSupportedImageFormats;
  static const std::vector<std::string> kSupportedPointCloudFormats;
};

using DataLoaderPtr = std::shared_ptr<DataLoader>;
using DataLoaderConstPtr = std::shared_ptr<const DataLoader>;