#pragma once

#include "merged_point_cloud.h"
#include <string>
#include <vector>
#include <memory>

/**
 * @file pcd_merger.h
 * @brief PCD文件合并器类定义
 */

namespace lidar_camera_calib {

/**
 * @brief 内存监控器
 *
 * 用于监控和限制PCD合并过程中的内存使用
 */
class MemoryMonitor {
 public:
  /**
   * @brief 构造函数
   */
  MemoryMonitor();

  /**
   * @brief 设置内存限制
   * @param limit_bytes 内存限制（字节）
   */
  void SetLimit(size_t limit_bytes);

  /**
   * @brief 检查当前内存使用是否超限
   * @throw MemoryLimitExceeded 如果超出内存限制
   */
  void CheckUsage() const;

  /**
   * @brief 获取当前内存使用量
   * @return 当前内存使用量（字节）
   */
  size_t GetCurrentUsage() const;

  /**
   * @brief 获取内存限制
   * @return 内存限制（字节）
   */
  size_t GetLimit() const;

  /**
   * @brief 检查是否设置了内存限制
   * @return true如果设置了内存限制
   */
  bool HasLimit() const;

 private:
  size_t limit_bytes_;
  bool has_limit_;
};

/**
 * @brief PCD合并错误基类
 */
class PcdMergeError : public std::exception {
 public:
  explicit PcdMergeError(const std::string& message);
  const char* what() const noexcept override;

 protected:
  std::string message_;
};

/**
 * @brief PCD文件损坏异常
 */
class PcdFileCorrupted : public PcdMergeError {
 public:
  explicit PcdFileCorrupted(const std::string& filename);
};

/**
 * @brief PCD格式不兼容异常
 */
class PcdFormatIncompatible : public PcdMergeError {
 public:
  PcdFormatIncompatible(const std::string& filename, const std::string& details);
};

/**
 * @brief 内存限制超出异常
 */
class MemoryLimitExceeded : public PcdMergeError {
 public:
  MemoryLimitExceeded(size_t current_usage, size_t limit);
};

/**
 * @brief 磁盘空间不足异常
 */
class InsufficientDiskSpace : public PcdMergeError {
 public:
  explicit InsufficientDiskSpace(const std::string& details);
};

/**
 * @brief PCD文件合并器
 *
 * 负责将多个PCD文件合并成单一点云，支持内存限制和降采样
 * @note 线程安全性：此类不是线程安全的，多线程使用需要外部同步
 * @note 内存管理：使用流式合并策略以控制内存使用
 */
class PcdMerger {
 public:
  /**
   * @brief 构造函数
   */
  PcdMerger();

  /**
   * @brief 析构函数
   */
  ~PcdMerger();

  /**
   * @brief 设置内存限制
   * @param limit_mb 内存限制（MB）
   */
  void SetMemoryLimit(size_t limit_mb);

  /**
   * @brief 设置最大文件数量限制
   * @param max_count 最大文件数量
   */
  void SetMaxFiles(int max_count);

  /**
   * @brief 设置降采样体素大小
   * @param size 体素大小（米），0表示不降采样
   */
  void SetDownsampleVoxelSize(float size);

  /**
   * @brief 合并指定的PCD文件列表
   * @param pcd_files PCD文件路径列表
   * @return MergedPointCloud 合并后的点云结构
   * @throw PcdMergeError 合并过程中的各种错误
   *
   * @note 合并流程：
   *       - 按提供的顺序处理文件
   *       - 验证每个PCD文件可读性
   *       - 跟踪内存使用并遵守限制
   *       - 返回包含统计信息的合并结果
   */
  MergedPointCloud MergeFiles(const std::vector<std::string>& pcd_files);

  /**
   * @brief 合并目录中的PCD文件
   * @param directory_path 目录路径
   * @param max_files 最大文件数量，-1表示不限制
   * @return MergedPointCloud 合并后的点云结构
   * @throw PcdMergeError 合并过程中的各种错误
   *
   * @note 执行流程：
   *       - 按字母顺序枚举PCD文件
   *       - 遵守max_files限制
   *       - 委托给MergeFiles()方法
   *       - 优雅处理目录不存在的情况
   */
  MergedPointCloud MergeDirectory(const std::string& directory_path, int max_files = -1);

  /**
   * @brief 估算内存使用量
   * @param pcd_files PCD文件路径列表
   * @return 估算的内存使用量（字节）
   *
   * @note 估算方式：
   *       - 返回估算的总内存使用量
   *       - 包含合并点云的开销
   *       - 快速完成（不完整读取文件）
   */
  size_t EstimateMemoryUsage(const std::vector<std::string>& pcd_files) const;

  /**
   * @brief 获取当前配置摘要
   * @return 包含内存限制、文件限制和降采样设置的摘要字符串
   */
  std::string GetConfigSummary() const;

  /**
   * @brief 重置合并器状态
   */
  void Reset();

 private:
  size_t memory_limit_mb_;                         // 内存限制（MB）
  int max_files_;                                  // 最大文件数量限制
  float downsample_voxel_size_;                    // 降采样体素大小
  std::unique_ptr<MemoryMonitor> memory_monitor_;  // 内存监控器

  /**
   * @brief 验证PCD文件可读性
   * @param filename PCD文件路径
   * @return true如果文件可读且格式有效
   * @throw PcdFileCorrupted 如果文件损坏
   * @throw PcdFormatIncompatible 如果格式不兼容
   */
  bool ValidatePcdFile(const std::string& filename) const;

  /**
   * @brief 加载单个PCD文件
   * @param filename PCD文件路径
   * @return 加载的点云指针
   * @throw PcdMergeError 加载失败时抛出
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr LoadPcdFile(const std::string& filename) const;

  /**
   * @brief 将源点云合并到目标点云
   * @param target 目标点云
   * @param source 源点云
   */
  void MergePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr target,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr source) const;

  /**
   * @brief 应用降采样
   * @param cloud 待降采样的点云
   * @return 降采样后的点云
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr ApplyDownsampling(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const;

  /**
   * @brief 获取目录中的PCD文件列表
   * @param directory_path 目录路径
   * @return 排序后的PCD文件路径列表
   */
  std::vector<std::string> GetPcdFilesInDirectory(const std::string& directory_path) const;

  /**
   * @brief 估算单个PCD文件的内存使用
   * @param filename PCD文件路径
   * @return 估算的内存使用量（字节）
   */
  size_t EstimateSingleFileMemory(const std::string& filename) const;

  /**
   * @brief 检查磁盘空间是否足够
   * @param required_space 需要的空间（字节）
   * @return true如果空间足够
   */
  bool CheckDiskSpace(size_t required_space) const;
};

}  // namespace lidar_camera_calib