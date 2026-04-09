/**
 * @file undistortion_preprocessor.h

 *
 * 该模块支持Kalibr完整相机模型集合的图像/视频去畸变预处理：
 * 相机投影模型: pinhole, omni, ds, eucm
 * 畸变模型: radtan, equi, fov, none
 * 支持所有有效的模型组合
 */

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include "config_manager.h"
// 前向声明
struct Params;

/**
 * @struct UndistortionConfig
 * @brief 去畸变配置结构
 */
struct UndistortionConfig {
  cv::Size image_size;            ///< 图像尺寸
  std::string input_dataset_dir;  ///< 输入数据集目录
  std::string target_camera_id;   ///< 目标相机ID (如"camera_0")，空表示所有相机
  std::string camera_model_path;  ///< 相机模型配置文件路径
  bool preserve_original;         ///< 是否保留原始数据

  // 计算得到的去畸变后内参
  std::vector<double> undistorted_intrinsics;

  UndistortionConfig() : preserve_original(true) {}
};

/**
 * @struct UndistortionResult
 * @brief 去畸变处理结果
 */
struct UndistortionResult {
  bool success;                                ///< 处理是否成功
  std::string error_message;                   ///< 错误信息
  std::vector<double> undistorted_intrinsics;  ///< 计算得到的新内参向量
  std::string camera_model_path;               ///< 相机模型配置文件路径
  cv::Mat image;                               ///< 原始图像
  cv::Mat undistort_image;                     ///< 去畸变后的图像

  UndistortionResult() : success(false) {}
};

/**
 * @class UndistortionPreprocessor
 * @brief 去畸变预处理器类
 *
 * 主要功能:
 * - 支持多种畸变模型的图像/视频去畸变
 * - 自动计算去畸变后的新内参
 * - 批量处理数据集中的视频文件
 * - 生成新的配置文件供后续标定使用
 *
 * @note 该类设计为线程安全
 */
class UndistortionPreprocessor {
 public:
  /**
   * @brief 构造函数
   */
  UndistortionPreprocessor();

  /**
   * @brief 析构函数
   */
  ~UndistortionPreprocessor();

  /**
   * @brief 处理整个数据集
   * @param config 去畸变配置
   * @return UndistortionResult 处理结果
   */
  bool ProcessDataset(const UndistortionConfig& config, UndistortionResult& result) const;
  /**
   * @brief 处理单个视频文件
   * @param input_video_path 输入视频路径
   * @param config 去畸变配置
   * @param image 去畸变后的图像
   * @return bool 处理是否成功
   */
  bool GetImageFromVideo(const std::string& input_video_path, const UndistortionConfig& config,
                         UndistortionResult& result) const;

  /**
   * @brief 处理单张图像
   * @param input_image 输入图像
   * @param output_image 输出图像
   * @param config 去畸变配置
   * @return bool 处理是否成功
   */
  bool ProcessImage(const cv::Mat& input_image, cv::Mat& output_image, const UndistortionConfig& config) const;

  /**
   * @brief 计算去畸变后的新内参向量
   * @param camera_model_path 相机模型
   * @param image_size 图像尺寸
   * @return std::vector<double> 新的内参向量
   */
  std::vector<double> ComputeUndistortedIntrinsics(const std::string& camera_model_path, cv::Size& image_size) const;

  /**
   * @brief 设置详细输出模式
   * @param verbose 是否启用详细输出
   */
  void SetVerbose(bool verbose);

  /**
   * @brief 获取最后的错误信息
   * @return std::string 错误信息
   */
  std::string GetLastError() const;

  /**
   * @brief 生成去畸变前后对比可视化图像
   * @param input_video_path 输入视频文件路径
   * @param config 去畸变配置
   * @param output_image_path 输出对比图像路径
   * @param frame_index 提取帧的索引 (默认为视频中间帧)
   * @return bool 生成是否成功
   */
  bool GenerateUndistortionVisualization(const std::string& input_video_path, const UndistortionConfig& config,
                                         const std::string& output_image_path, int frame_index = -1);

  /**
   * @brief 保存去畸变前后的对比图像到文件
   * @param original_image 原始图像
   * @param undistorted_image 去畸变后图像
   * @param undistorted_intrinsics 新内参向量
   * @param output_path 输出文件路径
   * @return bool 保存是否成功
   */
  bool SaveUndistortionComparison(const cv::Mat& original_image, const cv::Mat& undistorted_image,
                                  const std::vector<double>& undistorted_intrinsics, const std::string& output_path);

 private:
  bool verbose_;                    ///< 详细输出模式
  mutable std::string last_error_;  ///< 最后的错误信息
};

/**
 * @brief 去畸变预处理器智能指针类型定义
 */
using UndistortionPreprocessorPtr = std::shared_ptr<UndistortionPreprocessor>;