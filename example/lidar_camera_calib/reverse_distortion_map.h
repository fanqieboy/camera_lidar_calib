#ifndef REVERSE_DISTORTION_MAP_H
#define REVERSE_DISTORTION_MAP_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "camera_model/camera_models/Camera.h"  // Changed to base class

// 此函数用于生成从原始畸变图像到去畸变图像的反向映射
// 原理：对于原始图像上的每个点，计算它在去畸变图像上的对应位置
cv::Mat generateDistortionMapFromUndistorted(
    const camera_model::CameraConstPtr& camera_model,  // Changed to shared pointer to base class
    cv::Mat& reverse_map1,
    cv::Mat& reverse_map2,
    cv::Size undistorted_image_size,
    float fx,
    float fy,
    float cx,
    float cy,
    cv::Mat rmat
);

// 生成用于去畸变的映射表
// 此函数用于生成从去畸变图像到原始畸变图像的映射
// 原理：使用相机模型的 initUndistortRectifyMap 方法
cv::Mat generateUndistortionMap(
    const camera_model::CameraConstPtr& camera_model,  // Changed to shared pointer to base class
    cv::Mat& map1,
    cv::Mat& map2,
    cv::Size undistorted_image_size,
    float fx,
    float fy,
    float cx,
    float cy,
    cv::Mat rmat
);

// 从YAML文件加载相机参数
camera_model::CameraConstPtr loadCameraFromYaml(const std::string& yaml_path);

// 将cv::Mat映射表保存为CSV文件
void saveMapToCSV(const cv::Mat& map, const std::string& filename);

#endif // REVERSE_DISTORTION_MAP_H