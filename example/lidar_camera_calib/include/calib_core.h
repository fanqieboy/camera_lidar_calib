/**
 * @file calib_core.h
 * @brief 标定核心模块头文件
 *
 * 该文件定义了LiDAR-相机外参标定的核心算法类，包括单场景和多场景标定功能。
 * 使用SVD方法进行刚体变换估计，支持几何验证和数据质量检查。
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

// 前向声明
struct Params;

/**
 * @brief 刚体变换求解结果结构体
 *
 * 包含旋转矩阵、平移向量、变换矩阵等标定结果，以及误差评估和状态信息。
 */
struct RigidTransformResult {
  Eigen::Matrix3d rotation;        ///< 旋转矩阵 R
  Eigen::Vector3d translation;     ///< 平移向量 t
  Eigen::Matrix4d transformation;  ///< 4x4变换矩阵
  double rmse;                     ///< 均方根误差
  bool success;                    ///< 求解是否成功
  std::string error_message;       ///< 错误信息
  int num_points;                  ///< 用于标定的点数量

  RigidTransformResult() : rmse(0.0), success(false), num_points(0) {
    rotation = Eigen::Matrix3d::Identity();  // 初始化为单位矩阵
    translation = Eigen::Vector3d::Zero();   // 初始化为零向量
    transformation = Eigen::Matrix4d::Identity();  // 初始化为单位矩阵
  }
};

/**
 * @brief 多场景标定数据块结构体
 *
 * 包含单个场景的LiDAR和相机检测到的圆心坐标，用于多场景联合标定。
 */
struct CalibrationBlock {
  std::string timestamp;                                ///< 时间戳
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_centers;   ///< LiDAR检测圆心
  pcl::PointCloud<pcl::PointXYZI>::Ptr camera_centers;  ///< 相机检测圆心
  bool is_valid;                                        ///< 数据是否有效

  CalibrationBlock() : is_valid(false) {
    // 初始化点云指针
    lidar_centers = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    camera_centers = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  }
};

/**
 * @class CalibCore
 * @brief 标定核心算法类
 *
 * 主要功能点：
 * - 单场景标定：使用SVD方法估计刚体变换
 * - 多场景标定：联合优化多个场景的标定数据
 * - 几何验证：检查圆心几何形状是否符合标定板规格
 * - 数据质量检查：验证输入数据的有效性
 * - RMSE计算：评估标定精度
 *
 * @note 该类为线程安全类，支持并发使用
 * @see RigidTransformResult, CalibrationBlock
 */
class CalibCore {
 public:
  /**
   * @brief 构造函数
   * @param params 标定参数配置
   */
  explicit CalibCore(const Params& params);

  /**
   * @brief 析构函数
   */
  ~CalibCore();

  /**
   * @brief 执行单场景标定
   * @param lidar_centers LiDAR检测到的圆心坐标
   * @param camera_centers 相机检测到的圆心坐标
   * @return RigidTransformResult 标定结果，包含变换矩阵和RMSE
   * @note 要求输入点云大小相等且为4个点
   */
  RigidTransformResult PerformSingleSceneCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                                                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_centers);

  /**
   * @brief 执行多场景标定
   * @param calibration_blocks 多个标定数据块
   * @param weights 权重系数（可选，为nullptr则使用等权重）
   * @return RigidTransformResult 标定结果，包含变换矩阵和RMSE
   * @note 要求至少3个有效的标定数据块
   */
  RigidTransformResult PerformMultiSceneCalibration(const std::vector<CalibrationBlock>& calibration_blocks,
                                                    const std::vector<double>* weights = nullptr);

  /**
   * @brief 执行多场景标定（重载版本）
   * @param lidar_centers_list LiDAR圆心点云列表
   * @param camera_centers_list 相机圆心点云列表
   * @param weights 权重系数（可选，为nullptr则使用等权重）
   * @return RigidTransformResult 标定结果，包含变换矩阵和RMSE
   * @note 要求两个列表大小相等且至少包含3个点云
   */
  RigidTransformResult PerformMultiSceneCalibration(
      const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& lidar_centers_list,
      const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& camera_centers_list,
      const std::vector<double>* weights = nullptr);

  /**
   * @brief 计算两个点云之间的RMSE
   * @param cloud1 第一个点云
   * @param cloud2 第二个点云
   * @return double RMSE值，失败时返回-1.0
   * @note 要求两个点云大小相等且非空
   */
  static double ComputeRMSE(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2);

  /**
   * @brief 应用变换矩阵到点云
   * @param input_cloud 输入点云
   * @param output_cloud 输出点云（将被清空并重新填充）
   * @param transformation 4x4变换矩阵
   * @note 保持原有的intensity值不变
   */
  static void TransformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                                  const Eigen::Matrix4d& transformation);

  // /**
  //  * @brief 对圆心进行几何排序
  //  * @param input_centers 输入的圆心点云
  //  * @param sorted_centers 排序后的圆心点云
  //  * @param coordinate_system 坐标系类型（"camera"或"lidar"）
  //  * @note 要求输入点云包含4个点
  //  */
  // static void SortPatternCenters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_centers,
  //                                   pcl::PointCloud<pcl::PointXYZI>::Ptr& sorted_centers,
  //                                   const std::string& coordinate_system,
  //                                   const std::string& camera_id); // <-- 新增参数

  // /**
  //  * @brief 验证圆心几何形状是否符合标定板规格
  //  * @param centers 圆心点云
  //  * @param target_width 目标宽度
  //  * @param target_height 目标高度
  //  * @param tolerance 容差阈值（默认0.06）
  //  * @return bool 验证通过返回true，否则返回false
  //  */
  // static bool ValidatePatternGeometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr& centers, double target_width,
  //                                       double target_height, double tolerance,
  //                                       const std::string& camera_id);

  /**
   * @brief 保存标定数据到记录文件
   *
   * @param lidar_centers LiDAR圆心
   * @param camera_centers 相机圆心
   * @param output_path 输出路径
   * @param filename 文件名（默认为circle_center_record.txt）
   * @return bool 保存是否成功
   */
  static bool SaveCalibrationData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_centers,
                                  const std::string& output_path,
                                  const std::string& filename = "circle_center_record.txt");

  /**
   * @brief 从记录文件加载多场景标定数据
   *
   * @param record_file 记录文件路径
   * @param selected_indices 选择的数据索引（可选，为空则加载所有）
   * @return std::vector<CalibrationBlock> 标定数据块列表
   */
  static std::vector<CalibrationBlock> LoadCalibrationDataFromFile(
      const std::string& record_file, const std::vector<int>& selected_indices = std::vector<int>());

  /**
   * @brief 更新标定参数
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
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息
   */
  std::string GetLastError() const;

  /**
   * @brief 获取标定统计信息
   *
   * @return std::string 统计信息
   */
  std::string GetCalibrationStats() const;

 private:
  std::string camera_id_; // 【新增】存储当前相机ID

  // 相机方向参数
  int camera_direction_;  // 相机方向 (0:正前方,1:右侧,2:背后,3:左侧)

  /**
   * @brief 加权刚体变换求解（内部实现）
   *
   * @param lidar_points LiDAR点集
   * @param camera_points 相机点集
   * @param weights 权重（可选）
   * @return RigidTransformResult 求解结果
   */
  RigidTransformResult SolveWeightedRigidTransform(const std::vector<Eigen::Vector3d>& lidar_points,
                                                   const std::vector<Eigen::Vector3d>& camera_points,
                                                   const std::vector<double>* weights = nullptr) const;

  /**
   * @brief 解析圆心坐标行（用于文件读取）
   *
   * @param line 输入行
   * @param centers 输出的圆心坐标
   * @return bool 解析是否成功
   */
  static bool ParseCentersLine(const std::string& line, std::vector<Eigen::Vector3d>& centers);

  /**
   * @brief 验证输入数据的有效性
   *
   * @param lidar_centers LiDAR圆心
   * @param camera_centers 相机圆心
   * @return bool 数据是否有效
   */
  bool ValidateInputData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr& camera_centers) const;

 private:
  // 标定板参数
  double target_width_;        ///< 标定板宽度
  double target_height_;       ///< 标定板高度
  double geometry_tolerance_;  ///< 几何容差
  int expected_num_circles_;   ///< 期望的圆形数量

  // 状态和统计
  mutable std::string last_error_;  ///< 最后的错误信息
  bool verbose_;                    ///< 详细输出模式
  int total_calibrations_;          ///< 总标定次数
  int successful_calibrations_;     ///< 成功标定次数
  double total_processing_time_;    ///< 总处理时间

  // 常量
  static const int kTargetNumCircles = 4;         ///< 目标圆形数量
  static const double kDefaultGeometryTolerance;  ///< 默认几何容差
};

using CalibCorePtr = std::shared_ptr<CalibCore>;
