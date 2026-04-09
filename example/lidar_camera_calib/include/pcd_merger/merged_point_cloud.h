#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

/**
 * @file merged_point_cloud.h
 * @brief 合并点云数据结构定义
 */

namespace lidar_camera_calib {

/**
 * @brief 合并后的点云数据结构
 *
 * 包含合并后的点云数据、源文件信息和统计数据，支持多种查询操作
 */
struct MergedPointCloud {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;  // 合并后的点云
  std::vector<std::string> source_files;       // 源PCD文件列表
  std::vector<size_t> file_point_counts;       // 每个文件的点数统计

  /**
   * @brief 默认构造函数
   */
  MergedPointCloud();

  /**
   * @brief 构造函数，初始化点云指针
   * @param cloud_ptr 点云智能指针
   */
  explicit MergedPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);

  /**
   * @brief 获取合并后点云的总点数
   * @return 总点数
   */
  size_t GetTotalPoints() const;

  /**
   * @brief 获取合并操作摘要信息
   * @return 包含源文件数量和点数统计的摘要字符串
   */
  std::string GetMergeSummary() const;

  /**
   * @brief 验证点云数据完整性
   * @return true如果点云数据有效且一致
   *
   * @note 检查项目包括：
   *       - cloud指针非空
   *       - source_files和file_point_counts长度一致
   *       - 总点数与统计数据一致
   */
  bool Validate() const;

  /**
   * @brief 获取指定文件的点数
   * @param file_index 文件索引
   * @return 该文件的点数，索引无效时返回0
   */
  size_t GetFilePointCount(size_t file_index) const;

  /**
   * @brief 获取源文件数量
   * @return 合并时使用的源文件数量
   */
  size_t GetSourceFileCount() const;

  /**
   * @brief 检查点云是否为空
   * @return true如果点云为空或无效
   */
  bool IsEmpty() const;

  /**
   * @brief 清理点云数据
   *
   * 释放点云内存并清空统计信息
   */
  void Clear();

  /**
   * @brief 添加源文件统计信息
   * @param file_path 源文件路径
   * @param point_count 该文件的点数
   */
  void AddFileStatistics(const std::string& file_path, size_t point_count);
};

}  // namespace lidar_camera_calib