/**
 * @file common_lib.h
 * @brief 公共工具函数库头文件
 *
 * 该文件定义了项目中的公共工具函数，包括点云处理、图像投影、
 * 数据保存等功能。提供标定流程中通用的算法工具。
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "config_manager.h"
constexpr int kTargetNumCircles = 4;
// 前向声明
struct Params;
class QRDetector;
// 常量定义
const int kDebugMode = 1;
const double kGeometryTolerance = 0.06;

/**
 * @brief 计算两点云的RMSE
 * @param cloud1 第一个点云
 * @param cloud2 第二个点云
 * @return double RMSE值，失败时返回-1.0
 * @note 要求两个点云大小相等且非空
 */
double ComputeRmse(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2);

/**
 * @brief 将LiDAR点云转换到QR码坐标系
 * @param input_cloud 输入点云
 * @param output_cloud 输出点云（将被清空并重新填充）
 * @param transformation 变换矩阵
 */
void AlignPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud, const Eigen::Matrix4f& transformation);

/**
 * @brief 将点云投影到图像并着色
 * @param cloud 输入点云
 * @param transformation 变换矩阵
 * @param camera_matrix 相机内参矩阵
 * @param dist_coeffs 畸变系数
 * @param image 输入图像
 * @param colored_cloud 输出的着色点云
 */
void ProjectPointCloudToImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Matrix4f& transformation,
                              const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, const cv::Mat& image,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud);

/**
 * @brief 记录标定板孔中心
 * @param lidar_centers LiDAR检测的圆心
 * @param qr_centers QR码检测的圆心
 * @param params 标定参数
 */
void SaveTargetHoleCenters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& qr_centers, const Params& params);

/**
 * @brief 保存标定结果与着色点云
 * @param params 标定参数
 * @param transformation 变换矩阵
 * @param colored_cloud 着色点云
 * @param img_input 输入图像
 */
void SaveCalibrationResults(const Params& params, const Eigen::Matrix4f& transformation,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, const cv::Mat& img_input);

/**
 * @brief 记录标定板孔中心
 * @param lidar_centers LiDAR检测的圆心
 * @param qr_centers QR码检测的圆心
 * @param params 标定参数
 */
void SaveTargetHoleCenters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_centers,
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& qr_centers, const Params& params);

/**
 * @brief 对4个标定板中心点进行确定性排序。
 * @param pc 输入的无序点云 (4个点)。
 * @param v 存储排序后的点云。
 * @param camera_id 当前正在标定的相机ID，用于选择坐标变换。
 * @param input_coord_frame 输入点云的坐标系 ("lidar" 或 "camera")。
 */
void SortPatternCenters(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr v,
                        const std::string& input_coord_frame = "lidar");

void SaveCalibrationResults(const Params& params, const Eigen::Matrix4f& transformation,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, const cv::Mat& img_input);

/**
 * @brief 根据给定的方向编号，对点云进行绕Z轴的旋转。
 * @param input_cloud 输入点云
 * @param output_cloud 输出旋转后的点云
 * @param direction 旋转方向的编号 (0:正前方,1:背后,2:左侧,3:右侧)
 * @return bool 操作是否成功
 */
bool TransformLidarPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                          int direction);

/**
 * @brief 根据给定的方向编号，获取对应的修正旋转矩阵。
 * @param direction 安装方向编号 (0:正前方,1:右侧,2:背后,3:左侧)
 * @return Eigen::Matrix3d 3x3修正旋转矩阵
 */
Eigen::Matrix3d GetCorrectionRotationMatrix(int direction);

/**
 * @brief 对原始外参矩阵应用修正矩阵。
 * @param original_transform 原始外参矩阵
 * @param direction 安装方向编号 (0:正前方,1:右侧,2:背后,3:左侧)
 * @return Eigen::Matrix4d 修正后的外参矩阵
 */
Eigen::Matrix4d ApplyCorrectionToTransform(const Eigen::Matrix4d& original_transform, int direction);

/**
 * @brief 将 camera_direction 枚举值转换为对应的旋转角度（度，逆时针为正）。
 * @param direction 方向编号 (0→0°, 1→180°, 2→-90°, 3→90°)
 * @return double 对应旋转角度（度）
 */
double GetCameraDirectionAngle(int direction);

/**
 * @brief 根据任意角度对点云进行绕Z轴旋转（逆时针为正）。
 * @param input_cloud  输入点云
 * @param output_cloud 输出旋转后的点云
 * @param angle_deg    旋转角度（度，逆时针为正）
 * @return bool 操作是否成功
 */
bool TransformLidarPointsByAngle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
                                 double angle_deg);

/**
 * @brief 对外参矩阵按任意角度应用修正（右乘 R(angle_deg) 的4x4矩阵）。
 * @param original_transform 原始外参矩阵
 * @param angle_deg          修正角度（度，逆时针为正），含义与 TransformLidarPointsByAngle 相同
 * @return Eigen::Matrix4d 修正后的外参矩阵
 */
Eigen::Matrix4d ApplyCorrectionByAngle(const Eigen::Matrix4d& original_transform, double angle_deg);

/**
 * @brief 排序四个角点（相机/雷达坐标系）
 * @param pc 输入点云
 * @param v 排序后的点云
 * @param axis_mode 坐标系模式（"camera"或"lidar"）
 */

/**
 * @class Square
 * @brief 标定板正方形检测类
 *
 * 主要功能点：
 * - 从候选点中检测正方形图案
 * - 验证几何尺寸是否符合标定板规格
 * - 提供点间距离计算
 *
 * @note 该类用于标定板图案的几何验证
 * @see ValidatePatternGeometry
 */
class Square {
 public:
  /**
   * @brief 构造函数
   * @param candidates 候选点集
   * @param width 目标宽度
   * @param height 目标高度
   */
  Square(std::vector<pcl::PointXYZI> candidates, float width, float height);

  /**
   * @brief 计算两点间距离
   * @param pt1 第一个点
   * @param pt2 第二个点
   * @return float 距离值
   */
  float Distance(pcl::PointXYZI pt1, pcl::PointXYZI pt2);

  /**
   * @brief 获取指定索引的点
   * @param i 点索引
   * @return pcl::PointXYZI 指定点
   */
  pcl::PointXYZI At(int i);

  /**
   * @brief 检查正方形是否有效
   * @return bool 有效返回true，否则返回false
   */
  // 【修改】不再需要 QRDetector*，而是传入 camera_id
  bool IsValid(const std::string& camera_id);

 private:
  pcl::PointXYZI center_;                   ///< 中心点
  std::vector<pcl::PointXYZI> candidates_;  ///< 候选点集
  float target_width_;                      ///< 目标宽度
  float target_height_;                     ///< 目标高度
  float target_diagonal_;                   ///< 目标对角线长度
};

// 全局常量定义
extern const int kDebugMode;                ///< 调试模式开关
extern const double kGeometryTolerance;    ///< 几何容差
extern const int kTargetNumCircles;        ///< 目标圆形数量