/**
 * @file qr_detector.h
 * @brief QR/ArUco检测模块头文件 - 移除ROS依赖
 *
 * 该文件定义了独立的QR/ArUco标记检测器，支持从图像中检测ArUco标记，
 * 计算标定板位姿，并提取圆形标记的3D坐标。完全移除ROS依赖。
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE' file,
 * which is included as part of this source code package.
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <string>
#include <memory>

// 前向声明
struct Params;

/**
 * @struct QRDetectionResult
 * @brief QR/ArUco检测结果结构体
 *
 * 包含检测过程的所有相关信息:
 * - 检测成功状态和错误信息
 * - 标记数量和ID信息
 * - 标记角点和圆心3D坐标
 * - 标定板位姿信息
 * - 处理后的图像和检测置信度
 */
struct QRDetectionResult {
  bool success;                                          ///< 检测是否成功
  int markers_detected;                                  ///< 检测到的标记数量
  std::vector<int> marker_ids;                           ///< 标记ID列表
  std::vector<std::vector<cv::Point2f>> marker_corners;  ///< 标记角点
  pcl::PointCloud<pcl::PointXYZI>::Ptr circle_centers;   ///< 圆心3D坐标
  cv::Mat processed_image;                               ///< 处理后的图像（带标记）
  cv::Vec3d board_rvec;                                  ///< 标定板旋转向量
  cv::Vec3d board_tvec;                                  ///< 标定板平移向量
  std::string error_message;                             ///< 错误信息
  double detection_confidence;                           ///< 检测置信度

  QRDetectionResult()
      : success(false),
        markers_detected(0),
        circle_centers(new pcl::PointCloud<pcl::PointXYZI>()),  // 创建新的点云对象
        detection_confidence(0.0) {}
};

/**
 * @class QRDetector
 * @brief QR/ArUco检测器类 - 独立的图像检测模块
 *
 * 该类负责从图像中检测ArUco标记，计算标定板位姿，
 * 并提取圆形标记的3D坐标。完全移除ROS依赖。
 *
 * 主要功能:
 * - ArUco标记检测和识别
 * - 标定板位姿估计
 * - 圆形标记3D坐标提取
 * - 几何形状验证和筛选
 * - 可视化输出和错误处理
 *
 * @note 线程安全性: 该类不保证线程安全，需要外部同步
 * @see QRDetectionResult 检测结果结构体
 * @see Params 配置参数结构体
 */
class QRDetector {
 public:
  /**
   * @brief 构造函数
   *
   * @param params 标定参数配置
   * @throw std::invalid_argument 参数无效时抛出异常
   */
  explicit QRDetector(const Params& params);

  /**
   * @brief 析构函数
   */
  ~QRDetector();

  /**
   * @brief 从图像检测ArUco标记和圆心
   *
   * 这是主要的检测接口，执行完整的检测流程:
   * 1. ArUco标记检测
   * 2. 标定板位姿估计
   * 3. 圆心3D坐标计算
   * 4. 几何形状验证
   *
   * @param image 输入图像（灰度或彩色）
   * @return QRDetectionResult 包含所有检测信息的结果结构体
   * @note 输入图像不会被修改，处理结果存储在返回结构体中
   */
  QRDetectionResult DetectMarkers(const cv::Mat& image);

  /**
   * @brief 检测并提取圆心坐标（兼容原版接口）
   *
   * 简化版检测接口，仅返回成功状态和圆心坐标。
   * 内部调用DetectMarkers()并提取关键结果。
   *
   * @param[in] image 输入图像
   * @param[out] centers_cloud 输出的圆心点云
   * @return bool 检测是否成功
   * @note 兼容性接口，推荐使用DetectMarkers()获取完整信息
   */
  bool DetectQR(const cv::Mat& image, pcl::PointCloud<pcl::PointXYZI>::Ptr centers_cloud);

  /**
   * @brief 获取处理后的图像副本
   *
   * @return cv::Mat 带标记和标注的图像副本
   * @note 返回图像副本，不影响内部状态
   */
  cv::Mat GetProcessedImage() const;

  /**
   * @brief 获取相机内参矩阵
   *
   * @return cv::Mat 相机内参矩阵副本
   */
  cv::Mat GetCameraMatrix() const;

  /**
   * @brief 获取畸变系数
   *
   * @return cv::Mat 畸变系数副本
   */
  cv::Mat GetDistortionCoeffs() const;

  /**
   * @brief 更新相机参数
   *
   * @param params 新的参数配置
   * @throw std::invalid_argument 参数无效时抛出异常
   */
  void UpdateParameters(const Params& params);

  /**
   * @brief 设置详细输出模式
   *
   * @param verbose 是否启用详细输出
   */
  void SetVerbose(bool verbose);

  /**
   * @brief 设置检测参数
   *
   * @param corner_refinement 是否启用角点精化
   * @param adaptive_thresh_win_size_min 自适应阈值窗口大小最小值
   * @param adaptive_thresh_win_size_max 自适应阈值窗口大小最大值
   * @note 参数立即生效，影响后续检测操作
   */
  void SetDetectionParameters(bool corner_refinement = true, int adaptive_thresh_win_size_min = 3,
                              int adaptive_thresh_win_size_max = 23);


  /**
   * @brief 保存带标记的图像
   *
   * @param filepath 保存路径（绝对或相对路径）
   * @return bool 保存是否成功
   * @note 如果没有处理过的图像则返回false
   */
  bool SaveProcessedImage(const std::string& filepath) const;

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息字符串
   */
  std::string GetLastError() const;

  /**
   * @brief 获取检测统计信息
   *
   * @return std::string 包含检测成功率等统计信息的字符串
   */
  std::string GetDetectionStats() const;

 private:
  /**
   * @brief 投影3D点到图像平面（考虑畸变）
   *
   * @param pt_cv 3D点坐标
   * @param intrinsics 相机内参矩阵
   * @param dist_coeffs 畸变系数
   * @return cv::Point2f 投影后的2D点坐标
   */
  cv::Point2f ProjectPointWithDistortion(const cv::Point3f& pt_cv, const cv::Mat& intrinsics,
                                         const cv::Mat& dist_coeffs) const;

  /**
   * @brief 初始化相机参数
   *
   * @param params 参数配置
   */
  void InitializeCameraParameters(const Params& params);

  /**
   * @brief 创建标定板配置
   */
  void CreateBoardConfiguration();

  /**
   * @brief 生成候选组合
   *
   * @param n 总数量
   * @param k 选择数量
   * @param groups 输出的组合列表
   */
  void GenerateCombinations(int n, int k, std::vector<std::vector<int>>& groups) const;

  /**
   * @brief 验证候选圆心的几何形状
   *
   * @param candidates 候选点列表
   * @return double 几何验证得分（0.0-1.0，1.0为最佳）
   */
  double ValidateGeometry(const std::vector<pcl::PointXYZI>& candidates) const;

  /**
   * @brief 估计标定板位姿
   *
   * @param corners 检测到的角点
   * @param ids 标记ID列表
   * @param rvec 输出旋转向量
   * @param tvec 输出平移向量
   * @return bool 估计是否成功
   */
  bool EstimateBoardPose(const std::vector<std::vector<cv::Point2f>>& corners, const std::vector<int>& ids,
                         cv::Vec3d& rvec, cv::Vec3d& tvec) const;

 private:
  // 相机参数
  cv::Mat camera_matrix_;      ///< 相机内参矩阵
  cv::Mat distortion_coeffs_;  ///< 畸变系数

  // 标定板参数
  double marker_size_{};             ///< ArUco标记尺寸
  double delta_width_qr_center_{};   ///< QR中心宽度偏移
  double delta_height_qr_center_{};  ///< QR中心高度偏移
  double delta_width_circles_{};     ///< 圆形宽度偏移
  double delta_height_circles_{};    ///< 圆形高度偏移
  int min_detected_markers_{};       ///< 最小检测标记数

  // ArUco配置
  cv::Ptr<cv::aruco::Dictionary> dictionary_;          ///< ArUco字典
  cv::Ptr<cv::aruco::Board> board_;                    ///< 标定板
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;  ///< 检测参数

  // 标定板几何
  std::vector<std::vector<cv::Point3f>> board_corners_;  ///< 标定板角点
  std::vector<cv::Point3f> board_circle_centers_;        ///< 标定板圆心
  std::vector<int> board_ids_;                           ///< 标定板ID

  std::string camera_id_; // 【【【 新增成员变量 】】】

  // 状态
  cv::Mat processed_image_;         ///< 处理后的图像
  mutable std::string last_error_;  ///< 最后的错误信息
  bool verbose_;                    ///< 详细输出模式

  // 统计
  int total_detections_;       ///< 总检测次数
  int successful_detections_;  ///< 成功检测次数

  // 常量定义
  static constexpr double kGeometryTolerance = 0.15;  ///< 几何验证容差（15%）
};

using QRDetectorPtr = std::shared_ptr<QRDetector>;