/**
 * @file lidar_detector.h
 * @brief LiDAR点云检测模块头文件 - 独立的圆形标定板检测系统
 *
 * 该文件定义了独立的LiDAR点云检测系统，从点云中检测圆形标定板，
 * 通过滤波、平面分割、边界检测和圆形拟合等步骤实现高精度检测。
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 前向声明
struct Params;

/**
 * @brief LiDAR检测处理结果结构体
 *
 * 包含检测过程的所有结果和中间数据，用于调试、可视化和结果分析。
 */
struct LidarProcessingResult {
  bool success;                                         ///< 处理是否成功
  int circles_detected;                                 ///< 检测到的圆形数量
  pcl::PointCloud<pcl::PointXYZI>::Ptr circle_centers;  ///< 圆心3D坐标
  std::string error_message;                            ///< 错误信息
  double processing_time_ms;                            ///< 处理时间（毫秒）

  // 中间处理结果（用于调试和可视化）
  pcl::PointCloud<pcl::PointXYZI>::Ptr rotated_cloud;     ///< 按camera_direction+lidar_yaw_offset旋转后的原始点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;   ///< 滤波后的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud;      ///< 平面点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud;    ///< 对齐后的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud;       ///< 边界点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr center_z0_cloud;  ///< z=0平面的圆心
  pcl::PointCloud<pcl::PointXYZI>::Ptr noise_cloud;      ///< 噪声点云

  pcl::ModelCoefficients::Ptr plane_coefficients;           ///< 平面系数
  std::vector<pcl::ModelCoefficients> circle_coefficients;  ///< 圆形系数

  LidarProcessingResult()
      : success(false),
        circles_detected(0),
        circle_centers(new pcl::PointCloud<pcl::PointXYZI>()),  // 创建新的点云对象
        processing_time_ms(0.0),
        rotated_cloud(new pcl::PointCloud<pcl::PointXYZI>()),   // 创建旋转后点云对象
        filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>()), // 创建滤波后点云对象
        plane_cloud(new pcl::PointCloud<pcl::PointXYZI>()),    // 创建平面点云对象
        aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>()),  // 创建对齐后点云对象
        edge_cloud(new pcl::PointCloud<pcl::PointXYZI>()),     // 创建边界点云对象
        center_z0_cloud(new pcl::PointCloud<pcl::PointXYZI>()), // 创建z=0平面圆心对象
        noise_cloud(new pcl::PointCloud<pcl::PointXYZI>()),    // 创建噪声点云对象
        plane_coefficients(new pcl::ModelCoefficients()) {}    // 创建平面系数对象
};

/**
 * @class LidarDetector
 * @brief LiDAR检测器类 - 独立的点云检测和处理模块
 *
 * 主要功能:
 * - 点云滤波（强度和空间范围过滤）
 * - 平面分割（基于RANSAC算法）
 * - 点云对齐到z=0平面
 * - 边界点检测和提取
 * - 欧几里得聚类和圆形拟合
 * - 检测结果验证和统计
 *
 * @note 线程安全: 该类不保证线程安全，需要外部同步
 * @see LidarProcessingResult 检测结果结构体
 */
class LidarDetector {
 public:
  /**
   * @brief 构造函数
   *
   * @param params 标定参数
   */
  explicit LidarDetector(const Params& params);

  /**
   * @brief 析构函数
   */
  ~LidarDetector();

  /**
   * @brief 从点云检测圆形标定板
   *
   * @param cloud 输入点云数据
   * @return LidarProcessingResult 检测结果和中间数据
   */
  LidarProcessingResult DetectCircles(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

  /**
   * @brief 检测并提取圆心坐标（主要接口，兼容原版）
   *
   * @param cloud 输入点云数据
   * @param center_cloud 输出的圆心点云
   * @return bool 检测是否成功
   */
  bool DetectLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud);

  /**
   * @brief 更新检测参数
   *
   * @param params 新的参数配置
   */
  void UpdateParameters(const Params& params);

  /**
   * @brief 设置详细输出模式
   *
   * @param verbose 是否启用详细输出
   */
  void SetVerbose(bool verbose);

  /**
   * @brief 设置滤波参数
   *
   * @param voxel_size 体素滤波器尺寸
   * @param intensity_min 强度过滤最小值
   * @param intensity_max 强度过滤最大值
   */
  void SetFilterParameters(double voxel_size = 0.005, double intensity_min = 0.0, double intensity_max = 130.0);

  /**
   * @brief 设置平面分割参数
   *
   * @param distance_threshold RANSAC距离阈值
   * @param max_iterations 最大迭代次数
   */
  void SetPlaneSegmentationParameters(double distance_threshold = 0.01, int max_iterations = 1000);

  /**
   * @brief 设置边界检测参数
   *
   * @param radius_search 搜索半径
   * @param angle_threshold 角度阈值
   */
  void SetBoundaryDetectionParameters(double radius_search = 0.03, double angle_threshold = M_PI / 4);

  /**
   * @brief 设置聚类参数
   *
   * @param cluster_tolerance 聚类容差
   * @param min_cluster_size 最小聚类大小
   * @param max_cluster_size 最大聚类大小
   */
  void SetClusteringParameters(double cluster_tolerance = 0.02, int min_cluster_size = 50, int max_cluster_size = 1000);

  /**
   * @brief 设置圆形拟合参数
   *
   * @param distance_threshold RANSAC距离阈值
   * @param max_iterations 最大迭代次数
   * @param fitting_error_threshold 拟合错误阈值
   */
  void SetCircleFittingParameters(double distance_threshold = 0.01, int max_iterations = 1000,
                                  double fitting_error_threshold = 0.025);

  /**
   * @brief 保存中间结果点云
   *
   * @param result 处理结果数据
   * @param output_directory 输出目录路径
   * @param prefix 文件名前缀
   * @return bool 保存是否成功
   */
  bool SaveIntermediateResults(const LidarProcessingResult& result, const std::string& output_directory,
                               const std::string& prefix = "lidar_") const;

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息字符串
   */
  std::string GetLastError() const;

  /**
   * @brief 获取处理统计信息
   *
   * @return std::string 统计信息字符串
   */
  std::string GetProcessingStats() const;
  std::string camera_id_;

 private:
  /**
   * @brief 初始化参数
   *
   * @param params 参数配置结构体
   */
  void InitializeParameters(const Params& params);

  /**
   * @brief 点云滤波
   *
   * @param cloud 输入点云数据
   * @param filtered_cloud 输出滤波后的点云
   * @param noise_cloud 输出噪声点云
   * @return bool 滤波是否成功
   */
  bool FilterPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr noise_cloud);

  /**
   * @brief 平面分割
   *
   * @param cloud 输入点云数据
   * @param plane_cloud 输出平面点云
   * @param coefficients 输出平面系数
   * @return bool 分割是否成功
   */
  bool SegmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud,
                    pcl::ModelCoefficients::Ptr coefficients);

  /**
   * @brief 将平面点云对齐到z=0平面
   *
   * @param plane_cloud 输入平面点云
   * @param aligned_cloud 输出对齐后的点云
   * @param coefficients 平面系数参数
   * @param rotation_matrix 输出旋转矩阵
   * @param average_z 输出平均z值
   * @return bool 对齐是否成功
   */
  bool AlignPlaneToZ0(pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud, pcl::ModelCoefficients::Ptr coefficients,
                      Eigen::Matrix3d& rotation_matrix, double& average_z);

  /**
   * @brief 边界点检测
   *
   * @param cloud 输入点云数据
   * @param edge_cloud 输出边界点云
   * @return bool 检测是否成功
   */
  bool DetectBoundaryPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud);

  /**
   * @brief 聚类和圆形拟合
   *
   * @param edge_cloud 输入边界点云
   * @param center_z0_cloud 输出z=0平面的圆心
   * @param circle_coefficients 输出圆形系数
   * @return bool 拟合是否成功
   */
  bool ClusterAndFitCircles(pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr center_z0_cloud,
                            std::vector<pcl::ModelCoefficients>& circle_coefficients);

  /**
   * @brief 将z=0平面的圆心转换回原坐标系
   *
   * @param center_z0_cloud 输入z=0平面的圆心
   * @param center_cloud 输出原坐标系的圆心
   * @param rotation_matrix 旋转矩阵参数
   * @param average_z 平均z值
   */
  void TransformCentersBack(pcl::PointCloud<pcl::PointXYZI>::Ptr center_z0_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud, const Eigen::Matrix3d& rotation_matrix,
                            double average_z);

 private:
  // 滤波参数
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;  ///< 空间过滤范围
  double voxel_size_;                                     ///< 体素滤波器尺寸
  double intensity_min_, intensity_max_;                  ///< 强度过滤范围

  // 检测参数
  double circle_radius_;  ///< 圆形标定板半径

  // 平面分割参数
  double plane_distance_threshold_;  ///< RANSAC平面分割距离阈值
  int plane_max_iterations_;         ///< 平面分割最大迭代次数

  // 边界检测参数
  double boundary_radius_search_;    ///< 边界检测搜索半径
  double boundary_angle_threshold_;  ///< 边界检测角度阈值

  // 聚类参数
  double cluster_tolerance_;  ///< 欧几里得聚类容差
  int min_cluster_size_;      ///< 最小聚类大小
  int max_cluster_size_;      ///< 最大聚类大小

  // 圆形拟合参数
  double circle_distance_threshold_;  ///< RANSAC圆形拟合距离阈值
  int circle_max_iterations_;         ///< 圆形拟合最大迭代次数
  double fitting_error_threshold_;    ///< 圆形拟合误差阈值

  // 相机方向参数
  int camera_direction_;              ///< 相机方向 (0:正前方,1:背后,2:左侧,3:右侧)
  double lidar_yaw_offset_deg_;       ///< 雷达附加偏航角偏差（度），叠加在 camera_direction 之上

  // 状态管理
  mutable std::string last_error_;  ///< 最后的错误信息
  bool verbose_;                    ///< 详细输出模式

  // 统计信息
  int total_detections_;          ///< 总检测次数
  int successful_detections_;     ///< 成功检测次数
  double total_processing_time_;  ///< 总处理时间
};

// 类型别名定义
using LidarDetectorPtr = std::shared_ptr<LidarDetector>;