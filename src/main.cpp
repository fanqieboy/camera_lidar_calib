#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "calib_core.h"
#include "config_manager.h"
#include "data_loader.h"
#include "lidar_detector.h"
#include "common_lib.h"
#include "logger.h"
#include "qr_detector.h"

#define MAJOR_VERSION 1
#define MINOR_VERSION 0

int main(int argc, char** argv) {
    std::cout << "# INFO: Camera LiDAR calibration version: " << MAJOR_VERSION << "." << MINOR_VERSION << std::endl;

    // ========== 步骤 1: 加载配置文件 ==========
    std::string config_path = "../config/calib_config.yaml";
    if (argc >= 2) {
        config_path = argv[1];
    }

    std::cout << "\n[步骤 1/7] 加载配置文件: " << config_path << std::endl;

    auto& logger = Logger::GetInstance();
    logger.SetConsoleLevel(LogLevel::kInfo);

    ConfigManager config_manager;
    if (!config_manager.LoadConfig(config_path)) {
        std::cerr << "错误: 无法加载配置文件: " << config_path << std::endl;
        std::cerr << "详情: " << config_manager.GetLastError() << std::endl;
        return -1;
    }
    const Params& params = config_manager.GetParams();

    // ========== 步骤 2: 加载图像和点云数据 ==========
    std::cout << "\n[步骤 2/7] 加载图像和点云数据..." << std::endl;

    DataLoader data_loader;
    data_loader.SetVerbose(true);

    cv::Mat image = data_loader.LoadImage(params.image_path);
    if (image.empty()) {
        std::cerr << "错误: 图像加载失败: " << params.image_path << std::endl;
        return -1;
    }
    std::cout << "  ✓ 图像加载成功: " << image.cols << "x" << image.rows << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = data_loader.LoadPointCloud(params.pcd_path);
    if (!cloud || cloud->empty()) {
        std::cerr << "错误: 点云加载失败: " << params.pcd_path << std::endl;
        return -1;
    }
    std::cout << "  ✓ 点云加载成功: " << cloud->points.size() << " 个点" << std::endl;

    // ========== 步骤 3: 准备相机和雷达检测器 ==========
    std::cout << "\n[步骤 3/7] 准备相机和雷达检测器..." << std::endl;
    QRDetector camera_detector(params);
    LidarDetector lidar_detector(params);
    lidar_detector.SetVerbose(params.log_level == "DEBUG" || params.log_level == "INFO");
    std::cout << "[初始化器] 检测模块实例化完成" << std::endl;

    // ========== 步骤 4: 相机特征检测 ==========
    std::cout << "\n[步骤 4/7] 相机端特征检测 (识别 ArUco 标记并推算圆心)..." << std::endl;
    QRDetectionResult cam_result = camera_detector.DetectMarkers(image);
    if (!cam_result.success) {
        std::cerr << "相机特征检测失败: " << cam_result.error_message << std::endl;
        return -1;
    }
    std::cout << "  ✓ 相机端提取完成，圆心数量: " << cam_result.circle_centers->size() << std::endl;

    // 保存检测结果图像
    cv::Mat debug_image = camera_detector.GetProcessedImage();
    std::filesystem::create_directories(params.output_path);
    std::string output_img_path = params.output_path + "/camera_detection_result.png";
    if (!debug_image.empty() && cv::imwrite(output_img_path, debug_image)) {
        std::cout << "  ✓ 检测结果图像已保存至: " << output_img_path << std::endl;
    }

    // ========== 步骤 5: 雷达特征提取 ==========
    std::cout << "\n[步骤 5/7] 雷达端特征提取 (拟合圆心)..." << std::endl;
    LidarProcessingResult lidar_result = lidar_detector.DetectCircles(cloud);
    if (!lidar_result.success) {
        std::cerr << "雷达特征检测失败: " << lidar_result.error_message << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_lidar_centers(new pcl::PointCloud<pcl::PointXYZI>());
    ::SortPatternCenters(lidar_result.circle_centers, sorted_lidar_centers, "lidar");
    std::cout << "  ✓ 雷达端提取完成，拟合到的圆孔数量: " << lidar_result.circles_detected << std::endl;

    if (params.save_intermediate_results) {
        lidar_detector.SaveIntermediateResults(lidar_result, params.output_path, "test_");
    }

    // ========== 步骤 6: 特征点对照 ==========
    std::cout << "\n[步骤 6/7] 特征点对照:" << std::endl;
    if (cam_result.circle_centers->size() != sorted_lidar_centers->size()) {
        std::cerr << "错误: 特征点数量不匹配 (Camera: " << cam_result.circle_centers->size() 
                  << ", LiDAR: " << sorted_lidar_centers->size() << ")" << std::endl;
        return -1;
    }

    for (size_t i = 0; i < sorted_lidar_centers->size(); ++i) {
        const auto& lp = sorted_lidar_centers->points[i];
        const auto& cp = cam_result.circle_centers->points[i];
        printf("  点 [%zu]: LiDAR(%.3f, %.3f, %.3f) <-> Camera(%.3f, %.3f, %.3f)\n", 
               i, lp.x, lp.y, lp.z, cp.x, cp.y, cp.z);
    }

    // ========== 步骤 7: 执行外参解算 ==========
    std::cout << "\n[步骤 7/7] 开始执行外参解算..." << std::endl;
    CalibCore solver(params);
    RigidTransformResult calib_result = solver.PerformSingleSceneCalibration(sorted_lidar_centers, cam_result.circle_centers);

    if (calib_result.success) {
        std::cout << "\n✓ 标定成功！" << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "变换矩阵 (LiDAR -> Camera):\n" << calib_result.transformation << std::endl;
        std::cout << "RMSE (重投影误差): " << calib_result.rmse << " 米" << std::endl;
        std::cout << "参与标定的特征点对数: " << calib_result.num_points << std::endl;
        std::cout << "-------------------------------------------" << std::endl;

        // 保存点对数据到文本文件，方便以后加载和验证
        std::string record_file = params.output_path + "/circle_center_record.txt";
        CalibCore::SaveCalibrationData(sorted_lidar_centers, cam_result.circle_centers, params.output_path);
        std::cout << "  ✓ 标定数据对已保存至: " << record_file << std::endl;

        // 与合成数据真值对比 (仅在测试阶段有效)
        Eigen::Matrix4d ground_truth = Eigen::Matrix4d::Identity();
        ground_truth << 0, -1,  0,  0.0,
                        0,  0, -1, -0.15,
                        1,  0,  0,  0.0,
                        0,  0,  0,  1.0;
        
        double matrix_diff = (calib_result.transformation - ground_truth).norm();
        std::cout << "与真值的几何差异 (Frobenius norm): " << matrix_diff << std::endl;
    }
    else {
        std::cerr << "标定解算失败: " << calib_result.error_message << std::endl;
        return -1;
    }

    return 0;
}
