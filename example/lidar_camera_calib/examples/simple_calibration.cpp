/**
 * 
 * 展示如何使用 进行单场景 LiDAR-相机外参标定
 * 包含完整的数据加载、特征检测、标定计算和结果保存流程
 */

#pragma once

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "calib_core.h"
#include "config_manager.h"
#include "data_loader.h"
#include "lidar_detector.h"
#include "logger.h"
#include "qr_detector.h"
#include "result_exporter.h"

#ifdef ENABLE_VISUALIZATION
#include "visualizer.h"
#endif

/**
 * @brief 主程序入口函数
 * @param[in] argc 命令行参数数量
 * @param[in] argv 命令行参数数组
 * @return 程序退出码，0表示成功，非零表示失败
 * 
 * @note 程序需要至少3个参数：配置文件、图像文件、点云文件
 * @note 可选第4个参数指定输出目录，默认为 "output/"
 */
int main(int argc, char** argv) {
    std::cout << "==================================" << std::endl;
    std::cout << "LiDAR-Camera-Calib 简单标定示例" << std::endl;
    std::cout << "==================================" << std::endl;
    
    // 检查命令行参数
    if (argc < 4) {
        std::cout << "用法: " << argv[0] << " <config.yaml> <image.png> <cloud.pcd> [output_dir]" << std::endl;
        std::cout << "示例: " << argv[0] << " config/calib_config.yaml data/image.png data/cloud.pcd output/" << std::endl;
        return -1;
    }
    
    std::string config_path = argv[1];
    std::string image_path = argv[2];
    std::string cloud_path = argv[3];
    std::string output_dir = (argc > 4) ? argv[4] : "output/";
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 1. 初始化日志系统
        auto& logger = Logger::GetInstance();
        logger.SetLogLevel(LogLevel::kInfo);
        logger.EnableFileLogging(output_dir + "calibration.log");
        
        logger.Info("开始标定流程...");
        logger.Info("配置文件: %s", config_path.c_str());
        logger.Info("图像文件: %s", image_path.c_str());
        logger.Info("点云文件: %s", cloud_path.c_str());
        logger.Info("输出目录: %s", output_dir.c_str());
        
        // 2. 加载配置
        std::cout << "\n[1/7] 加载配置文件..." << std::endl;
        ConfigManager config_manager;
        
        if (!config_manager.LoadConfig(config_path)) {
            std::cerr << "错误: 无法加载配置文件 " << config_path << std::endl;
            std::cerr << "详情: " << config_manager.GetLastError() << std::endl;
            return -1;
        }
        
        const Params& params = config_manager.GetParams();
        std::cout << "✓ 配置加载成功" << std::endl;
        config_manager.PrintParams();
        
        // 3. 加载数据
        std::cout << "\n[2/7] 加载标定数据..." << std::endl;
        DataLoader data_loader;
        data_loader.SetVerbose(true);
        
        cv::Mat image = data_loader.LoadImage(image_path);
        if (image.empty()) {
            std::cerr << "错误: 无法加载图像 " << image_path << std::endl;
            std::cerr << "详情: " << data_loader.GetLastError() << std::endl;
            return -1;
        }
        std::cout << "✓ 图像加载成功 (" << image.cols << "x" << image.rows << ")" << std::endl;
        
        auto point_cloud = data_loader.LoadPointCloud(cloud_path);
        if (!point_cloud || point_cloud->points.empty()) {
            std::cerr << "错误: 无法加载点云 " << cloud_path << std::endl;
            std::cerr << "详情: " << data_loader.GetLastError() << std::endl;
            return -1;
        }
        std::cout << "✓ 点云加载成功 (" << point_cloud->points.size() << " 点)" << std::endl;
        
        // 4. 配置检测器
        std::cout << "\n[3/7] 初始化检测器..." << std::endl;
        
        QRDetector qr_detector;
        if (!qr_detector.Configure(params)) {
            std::cerr << "错误: QR 检测器配置失败" << std::endl;
            std::cerr << "详情: " << qr_detector.GetLastError() << std::endl;
            return -1;
        }
        std::cout << "✓ QR 检测器初始化完成" << std::endl;
        
        LidarDetector lidar_detector;
        if (!lidar_detector.Configure(params)) {
            std::cerr << "错误: LiDAR 检测器配置失败" << std::endl;
            std::cerr << "详情: " << lidar_detector.GetLastError() << std::endl;
            return -1;
        }
        std::cout << "✓ LiDAR 检测器初始化完成" << std::endl;
        
        // 5. 图像检测
        std::cout << "\n[4/7] 执行图像标记检测..." << std::endl;
        auto qr_result = qr_detector.DetectMarkers(image);
        
        if (!qr_result.success) {
            std::cerr << "错误: 图像标记检测失败" << std::endl;
            std::cerr << "详情: " << qr_detector.GetLastError() << std::endl;
            return -1;
        }
        
        std::cout << "✓ 检测到 " << qr_result.marker_centers.size() << " 个标记" << std::endl;
        for (size_t i = 0; i < qr_result.marker_centers.size(); ++i) {
            std::cout << "  标记 " << qr_result.marker_ids[i] 
                     << ": (" << qr_result.marker_centers[i].x 
                     << ", " << qr_result.marker_centers[i].y << ")" << std::endl;
        }
        
        // 6. 点云检测
        std::cout << "\n[5/7] 执行点云圆形检测..." << std::endl;
        auto lidar_result = lidar_detector.DetectCircles(point_cloud);
        
        if (!lidar_result.success) {
            std::cerr << "错误: 点云圆形检测失败" << std::endl;
            std::cerr << "详情: " << lidar_detector.GetLastError() << std::endl;
            return -1;
        }
        
        std::cout << "✓ 检测到 " << lidar_result.circle_centers.size() << " 个圆形" << std::endl;
        for (size_t i = 0; i < lidar_result.circle_centers.size(); ++i) {
            std::cout << "  圆形 " << i 
                     << ": (" << lidar_result.circle_centers[i].x 
                     << ", " << lidar_result.circle_centers[i].y << ")"
                     << " 半径: " << lidar_result.circle_radii[i] << std::endl;
        }
        
        // 7. 执行标定
        std::cout << "\n[6/7] 执行标定计算..." << std::endl;
        
        // 准备标定数据
        SceneCalibData calib_data;
        calib_data.scene_name = "simple_example";
        calib_data.image_points = qr_result.marker_centers;
        
        // 匹配点对数量
        size_t min_points = std::min(qr_result.marker_centers.size(), lidar_result.circle_centers.size());
        if (min_points < params.min_detected_markers) {
            std::cerr << "错误: 检测到的对应点对数量不足 (" << min_points 
                     << " < " << params.min_detected_markers << ")" << std::endl;
            return -1;
        }
        
        // 转换 LiDAR 检测结果为 3D 点
        for (size_t i = 0; i < min_points; ++i) {
            cv::Point3f lidar_point(
                lidar_result.circle_centers[i].x,
                lidar_result.circle_centers[i].y,
                2.0f  // 假设 Z 坐标（实际应该从平面分割获得）
            );
            calib_data.lidar_points.push_back(lidar_point);
        }
        
        // 调整图像点数量以匹配
        calib_data.image_points.resize(min_points);
        calib_data.is_valid = true;
        
        std::cout << "准备标定，使用 " << min_points << " 对对应点" << std::endl;
        
        // 初始化标定核心
        CalibCore calib_core;
        if (!calib_core.Configure(params)) {
            std::cerr << "错误: 标定核心初始化失败" << std::endl;
            std::cerr << "详情: " << calib_core.GetLastError() << std::endl;
            return -1;
        }
        
        // 执行标定
        auto calib_result = calib_core.CalibrateSingleScene(calib_data);
        
        if (!calib_result.success) {
            std::cerr << "错误: 标定计算失败" << std::endl;
            std::cerr << "详情: " << calib_core.GetLastError() << std::endl;
            return -1;
        }
        
        std::cout << "✓ 标定成功！" << std::endl;
        std::cout << "  RMSE: " << calib_result.rmse << " 像素" << std::endl;
        std::cout << "  点对数量: " << calib_result.point_pairs << std::endl;
        
        // 8. 保存结果
        std::cout << "\n[7/7] 保存标定结果..." << std::endl;
        
        // 创建输出目录
        std::filesystem::create_directories(output_dir);
        
        ResultExporter result_exporter;
        
        // 保存多种格式
        bool save_success = true;
        
        std::string txt_path = output_dir + "calibration_result.txt";
        if (!result_exporter.ExportTXT(txt_path, calib_result)) {
            std::cerr << "警告: 无法保存 TXT 格式结果" << std::endl;
            save_success = false;
        } else {
            std::cout << "✓ TXT 结果已保存到: " << txt_path << std::endl;
        }
        
        std::string yaml_path = output_dir + "calibration_result.yaml";
        if (!result_exporter.ExportYAML(yaml_path, calib_result)) {
            std::cerr << "警告: 无法保存 YAML 格式结果" << std::endl;
            save_success = false;
        } else {
            std::cout << "✓ YAML 结果已保存到: " << yaml_path << std::endl;
        }
        
        std::string json_path = output_dir + "calibration_result.json";
        if (!result_exporter.ExportJSON(json_path, calib_result)) {
            std::cerr << "警告: 无法保存 JSON 格式结果" << std::endl;
            save_success = false;
        } else {
            std::cout << "✓ JSON 结果已保存到: " << json_path << std::endl;
        }
        
        // 保存着色点云（如果可能）
        if (lidar_result.filtered_cloud && !lidar_result.filtered_cloud->points.empty()) {
            std::string pcd_path = output_dir + "colored_pointcloud.pcd";
            if (result_exporter.ExportPointCloud(pcd_path, lidar_result.filtered_cloud)) {
                std::cout << "✓ 着色点云已保存到: " << pcd_path << std::endl;
            }
        }
        
#ifdef ENABLE_VISUALIZATION
        // 可视化结果（可选）
        std::cout << "\n是否显示可视化结果？ (y/n): ";
        char choice;
        std::cin >> choice;
        
        if (choice == 'y' || choice == 'Y') {
            std::cout << "启动可视化..." << std::endl;
            
            Visualizer visualizer;
            if (visualizer.Initialize("LiDAR-Camera-Calib 标定结果")) {
                // 显示原始点云
                visualizer.ShowPointCloud(point_cloud, "original_cloud");
                
                // 显示过滤后的点云
                if (lidar_result.filtered_cloud) {
                    visualizer.ShowPointCloud(lidar_result.filtered_cloud, "filtered_cloud");
                }
                
                // 显示图像
                visualizer.ShowImageWithFeatures(image, qr_result.marker_centers, "detected_markers");
                
                std::cout << "可视化窗口已打开，按 'q' 退出..." << std::endl;
                
                // 交互式显示
                while (!visualizer.WasStopped()) {
                    visualizer.SpinOnce(100);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                
                visualizer.Close();
            } else {
                std::cerr << "警告: 无法启动可视化" << std::endl;
            }
        }
#endif
        
        // 计算总耗时
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "\n==================================" << std::endl;
        std::cout << "标定完成！" << std::endl;
        std::cout << "总耗时: " << duration.count() << " 毫秒" << std::endl;
        std::cout << "结果目录: " << output_dir << std::endl;
        std::cout << "==================================" << std::endl;
        
        logger.Info("标定成功完成，耗时 %ld 毫秒", duration.count());
        
        // 显示变换矩阵
        std::cout << "\n变换矩阵 (LiDAR -> Camera):" << std::endl;
        for (int i = 0; i < 4; ++i) {
            std::cout << "  ";
            for (int j = 0; j < 4; ++j) {
                std::cout << std::setw(12) << std::setprecision(6) << std::fixed 
                         << calib_result.transform_matrix(i, j);
            }
            std::cout << std::endl;
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "未知异常发生" << std::endl;
        return -1;
    }
}