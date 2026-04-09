/**
 * @file main_standalone.cpp

 * 该文件提供了标定工具的独立版本主程序入口，
 *
 *
 * 主要功能：
 * - 命令行参数解析和验证
 * - 配置文件加载和管理
 * - 数据加载和预处理
 * - ArUco 标记检测
 * - LiDAR 圆形目标检测
 * - 单场景/多场景标定执行
 * - 可视化结果展示
 * - 标定结果导出
 *
 * @note 该版本为线程安全的独立程序
 * @see CalibCore, ConfigManager, DataLoader
 */

#include <iomanip>
#include <iostream>
#include <memory>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "calib_core.h"
#include "cli.h"
#include "common_lib.h"
#include "config_manager.h"
#include "data_loader.h"
#include "lidar_detector.h"
#include "qr_detector.h"
#include "result_exporter.h"
#include "visualizer.h"
#include "dataset_loader.h"
#include "pcd_merger.h"
#include "video_extractor.h"
#include "exception_handler.h"

// 去畸变预处理模块 (v2.2+)
#include "undistortion_preprocessor.h"

// 常量定义
namespace {
constexpr int kMinScenesForMultiCalibration = 3;  ///< 多场景标定所需的最少场景数量
constexpr int kVisualizationSpinTime = 100;       ///< 可视化旋转等待时间（毫秒）
}  // namespace

/**
 * @class CalibrationApp
 * @brief LiDAR-Camera-Calib 主应用程序类
 *
 * 该类封装了标定应用程序的完整流程，包括：
 * - 命令行参数解析
 * - 配置管理
 * - 数据加载
 * - 检测算法
 * - 标定算法
 * - 结果可视化和导出
 *
 * @note 该类设计为单例使用，线程安全
 * @see CLI, ConfigManager, CalibCore
 */
class CalibrationApp {
 public:
  CalibrationApp()
      : cli_(std::make_shared<Cli>()),
        config_manager_(nullptr),
        data_loader_(nullptr),
        qr_detector_(nullptr),
        lidar_detector_(nullptr),
        calib_core_(nullptr),
        visualizer_(nullptr),
        result_exporter_(nullptr),
        dataset_loader_(nullptr),
        pcd_merger_(nullptr),
        video_extractor_(nullptr),
        undistortion_preprocessor_(nullptr) {}

  /**
   * @brief 运行主应用程序
   * @param argc [in] 命令行参数个数
   * @param argv [in] 命令行参数数组
   * @return int 程序退出状态码（0表示成功，非0表示失败）
   * @throw std::exception 当发生不可恢复错误时抛出
   */
  int Run(int argc, char* argv[]) {
    try {
      // 解析命令行参数
      auto options = cli_->ParseArguments(argc, argv);

      if (options.help) {
        cli_->ShowHelp(argv[0]);
        return 0;
      }

      if (options.version) {
        cli_->ShowVersion();
        return 0;
      }

      // 显示配置摘要
      if (!options.quiet) {
        cli_->ShowOptionsSummary(options);
      }

      // 空运行模式
      if (options.dry_run) {
        std::cout << "Dry run mode - would execute the calibration with above settings." << std::endl;
        return 0;
      }

      // 初始化组件
      if (!Initialize(options)) {
        std::cerr << "Failed to initialize application components" << std::endl;
        return 1;
      }

      // 根据模式执行不同的操作
      int result = 0;
      switch (options.mode) {
        case CliOptions::Mode::kSingleScene:
          result = RunSingleSceneCalibration(options);
          break;
        case CliOptions::Mode::kMultiScene:
          result = RunMultiSceneCalibration(options);
          break;
        case CliOptions::Mode::kProcessOnly:
          result = RunProcessOnly(options);
          break;
        case CliOptions::Mode::kBatchProcess:
          result = RunBatchProcess(options);
          break;
        case CliOptions::Mode::kInteractive:
          result = RunInteractive(options);
          break;
        default:
          std::cerr << "Unknown operation mode" << std::endl;
          result = 1;
          break;
      }

      return result;
    } catch (const std::exception& e) {
      std::cerr << "Application error: " << e.what() << std::endl;
      return 1;
    }
  }

 private:
  /**
   * @brief 初始化应用程序组件
   */
  bool Initialize(const CliOptions& options) {
    try {
      config_manager_ = std::make_shared<ConfigManager>();
      std::string config_file =
          options.config_file.empty() ? "config/calib_config_standalone.yaml" : options.config_file;

      // 【关键】加载【原始】配置文件
      if (!config_manager_->LoadConfig(config_file)) {
        std::cerr << "Warning: Could not load config file, using defaults" << std::endl;
      }

      const Params& params = config_manager_->GetParams();

      data_loader_ = std::make_shared<DataLoader>();

      // 【关键】所有组件都用【原始】参数进行【第一次】初始化
      qr_detector_ = std::make_shared<QRDetector>(params);
      lidar_detector_ = std::make_shared<LidarDetector>(params);
      calib_core_ = std::make_shared<CalibCore>(params);

      // 初始化可视化器（如果启用）
      if (params.enable_visualization) {
        visualizer_ = std::make_shared<Visualizer>(params);
        visualizer_->SetVerbose(options.verbose);

        // 为单场景标定设置数据集名称
        if (!params.dataset_dir.empty()) {
          // 从dataset_dir提取数据集名称
          std::string dataset_name = "unknown_dataset";
          size_t last_slash = params.dataset_dir.find_last_of("/\\");
          if (last_slash != std::string::npos) {
            dataset_name = params.dataset_dir.substr(last_slash + 1);
          } else {
            dataset_name = params.dataset_dir;
          }

          // 移除可能的路径相对前缀
          if (dataset_name.substr(0, 2) == "./") {
            dataset_name = dataset_name.substr(2);
          }

          // 更新可视化器配置
          VisualizationConfig vis_config = visualizer_->GetVisualizationConfig();
          vis_config.output_directory = params.output_path.empty() ? "./output" : params.output_path;
          vis_config.dataset_name = dataset_name;
          visualizer_->SetVisualizationConfig(vis_config);

          if (options.verbose) {
            std::cout << "[CalibrationApp] Set dataset name for visualizer: " << dataset_name << std::endl;
          }
        }
      }

      // 初始化结果输出器
      ExportConfig export_config;
      export_config.output_directory = params.output_path.empty() ? "./output" : params.output_path;
      export_config.filename_prefix = options.output_prefix.empty() ? "calib_result" : options.output_prefix;
      export_config.save_intermediate_results = params.save_intermediate_results;

      // 设置输出格式
      if (!options.output_formats.empty()) {
        export_config.enabled_formats.clear();
        for (const auto& format : options.output_formats) {
          if (format == "txt")
            export_config.enabled_formats.push_back(OutputFormat::kTxt);
          else if (format == "yaml")
            export_config.enabled_formats.push_back(OutputFormat::kYaml);
          else if (format == "json")
            export_config.enabled_formats.push_back(OutputFormat::kJson);
          else if (format == "xml")
            export_config.enabled_formats.push_back(OutputFormat::kXml);
          else if (format == "csv")
            export_config.enabled_formats.push_back(OutputFormat::kCsv);
        }
      }

      result_exporter_ = std::make_shared<ResultExporter>(export_config);
      result_exporter_->SetVerbose(options.verbose);

      // 初始化新数据集处理组件 (Feature 001-1-pcd-pcd)
      if (!params.dataset_dir.empty()) {
        // 初始化数据集加载器
        dataset_loader_ = std::make_shared<lidar_camera_calib::DatasetLoader>(params.dataset_dir);
        if (!params.camera_id.empty()) {
          dataset_loader_->SetCameraId(params.camera_id);
        }
        if (params.max_pcds > 0) {
          dataset_loader_->SetMaxPcdFiles(params.max_pcds);
        }

        // 初始化PCD合并器
        pcd_merger_ = std::make_shared<lidar_camera_calib::PcdMerger>();
        if (params.max_pcds > 0) {
          pcd_merger_->SetMaxFiles(params.max_pcds);
        }

        // 初始化视频提取器
        video_extractor_ = std::make_shared<lidar_camera_calib::VideoExtractor>();
        video_extractor_->SetOutputFormat("jpg");
        video_extractor_->SetQualityFactor(95);
      }

      // 设置全局异常处理器
      lidar_camera_calib::g_exception_handler.SetVerbose(options.verbose);
      lidar_camera_calib::g_exception_handler.SetQuiet(options.quiet);

      // 初始化去畸变预处理器 (v2.2+)
      undistortion_preprocessor_ = std::make_shared<UndistortionPreprocessor>();
      undistortion_preprocessor_->SetVerbose(options.verbose);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "Initialization error: " << e.what() << std::endl;
      return false;
    }
  }

  /**
   * @brief 运行单场景标定
   */
  int RunSingleSceneCalibration(const CliOptions& options) {
    if (!options.quiet) {
      std::cout << "Running single scene calibration..." << std::endl;
    }

    try {
      // 【第1步：获取初始参数】
      auto params_ptr = std::make_shared<Params>(config_manager_->GetParams());
      UndistortionResult undistort_result;

      if (!options.quiet) {
        std::cout << "\n=== 开始执行去畸变预处理... ===" << std::endl;
      }

      // [修改] 调用我们修改过的新函数，传入用户选择的模型
      int preprocessing_status = RunUndistortionPreprocessing(undistort_result);

      if (preprocessing_status != 0) {
        std::cerr << "去畸变预处理失败，标定终止。" << std::endl;
        return 1;
      }

      // 【第3步：创建全新的、完全正确的参数对象】
      if (!options.quiet) {
        std::cout << "\n=== 使用去畸变后的新参数重新配置标定流程 ===" << std::endl;
      }

      // 3.2 在内存中，用预处理结果彻底更新它
      params_ptr->fx = undistort_result.undistorted_intrinsics[0];
      params_ptr->fy = undistort_result.undistorted_intrinsics[1];
      params_ptr->cx = undistort_result.undistorted_intrinsics[2];
      params_ptr->cy = undistort_result.undistorted_intrinsics[3];

      // 3.4 使用这个全新的、全局生效的参数对象，重新初始化所有相关组件
      qr_detector_ = std::make_shared<QRDetector>(*params_ptr);
      dataset_loader_ = std::make_shared<lidar_camera_calib::DatasetLoader>(params_ptr->dataset_dir);

      // 确保 camera_id 被传递
      if (!params_ptr->camera_id.empty()) {
        dataset_loader_->SetCameraId(params_ptr->camera_id);
      }

      // 【第4步：后续所有流程都使用最终确定的参数】
      const Params& params = *params_ptr;

      if (params.dataset_dir.empty()) {
        std::cerr << "Error: No valid dataset directory specified." << std::endl;
        return 1;
      }

      // 加载数据 (现在会从正确的目录加载)
      SceneData scene_data = RunDatasetProcessing(options);
      // 将去畸变的图片传进来
      scene_data.image = undistort_result.undistort_image;
      if (!scene_data.is_valid) {
        std::cerr << "Dataset processing failed from directory: " << params.dataset_dir << std::endl;
        return 1;
      }

      // QR检测 (现在会使用正确的相机参数，处理正确的图像)
      auto qr_result = qr_detector_->DetectMarkers(scene_data.image);
      if (!qr_result.success) {
        std::cerr << "QR detection failed: " << qr_result.error_message << std::endl;
        return 1;
      }

      // LiDAR检测
      auto lidar_result = lidar_detector_->DetectCircles(scene_data.point_cloud);
      if (!lidar_result.success) {
        std::cerr << "LiDAR detection failed: " << lidar_result.error_message << std::endl;

        // 如果启用可视化，显示失败的LiDAR检测中间结果
        if (visualizer_) {
          if (!options.quiet) {
            std::cout << "Showing failed LiDAR processing results for diagnosis..." << std::endl;
          }
          visualizer_->ShowFailedLidarProcessing(lidar_result, scene_data.point_cloud);
          if (!options.quiet) {
            std::cout << "Press any key to close visualization..." << std::endl;
            cv::waitKey(0);
            cv::destroyAllWindows();
          }
        }
        return 1;
      }

      // 执行标定
      auto calib_result =
          calib_core_->PerformSingleSceneCalibration(lidar_result.circle_centers, qr_result.circle_centers);

      if (!calib_result.success) {
        std::cerr << "Calibration failed: " << calib_result.error_message << std::endl;

        // 如果启用可视化，显示标定失败时的输入数据
        if (visualizer_) {
          if (!options.quiet) {
            std::cout << "Showing calibration input data for diagnosis..." << std::endl;
          }
          visualizer_->ShowFailedCalibration(qr_result, lidar_result, scene_data.image, scene_data.point_cloud,
                                             qr_detector_->GetCameraMatrix(), qr_detector_->GetDistortionCoeffs());
          if (!options.quiet) {
            std::cout << "Press any key to close visualization..." << std::endl;
            cv::waitKey(0);
            cv::destroyAllWindows();
          }
        }
        return 1;
      }

      // 显示原始标定结果（伪外参）
      if (!options.quiet) {
        std::cout << "Calibration successful!" << std::endl;
        std::cout << "RMSE: " << std::fixed << std::setprecision(4) << calib_result.rmse << " m" << std::endl;
        if (params.camera_direction != 0) {
          std::cout << "Raw (pseudo) Transformation matrix (before correction):" << std::endl;
        } else {
          std::cout << "Lidar to Camera Transformation matrix:" << std::endl;
        }
        std::cout << calib_result.transformation << std::endl;
      }

      // 根据 camera_direction + lidar_yaw_offset_deg 右乘修正矩阵，得到真实外参
      if (params.camera_direction != 0 || params.lidar_yaw_offset_deg != 0.0) {
        double total_yaw = GetCameraDirectionAngle(params.camera_direction) + params.lidar_yaw_offset_deg;
        if (params.lidar_yaw_offset_deg != 0.0) {
          // 有附加偏差：用连续角度修正
          calib_result.transformation = ApplyCorrectionByAngle(calib_result.transformation, total_yaw);
        } else {
          // 无附加偏差：沿用原有离散修正逻辑
          calib_result.transformation = ApplyCorrectionToTransform(calib_result.transformation, params.camera_direction);
        }
        // 同步更新旋转矩阵和平移向量，保持结构体内部一致
        calib_result.rotation    = calib_result.transformation.block<3, 3>(0, 0);
        calib_result.translation = calib_result.transformation.block<3, 1>(0, 3);
        if (!options.quiet) {
          std::cout << "Corrected Lidar to Camera Transformation matrix (camera_direction="
                    << params.camera_direction << ", lidar_yaw_offset_deg=" << params.lidar_yaw_offset_deg
                    << ", total=" << total_yaw << "deg):" << std::endl;
          std::cout << calib_result.transformation << std::endl;
        }
      }

      // 可视化
      if (visualizer_) {
        visualizer_->ShowLidarProcessingResult(lidar_result, params.save_intermediate_results);
        visualizer_->ShowQrDetectionResult(qr_result, true);
        visualizer_->ShowCalibrationResult(calib_result, scene_data.point_cloud, scene_data.image,
                                           qr_detector_->GetCameraMatrix(), qr_detector_->GetDistortionCoeffs());

        if (!options.quiet) {
          std::cout << "Press any key to close visualization..." << std::endl;
          cv::waitKey(0);
          cv::destroyAllWindows();
        }
      }

      // 导出结果
      CalibrationResultPackage result_package;
      result_package.transformation_result = calib_result;
      result_package.lidar_centers = lidar_result.circle_centers;
      result_package.camera_centers = qr_result.circle_centers;
      result_package.camera_image = scene_data.image;
      result_package.processed_image = qr_result.processed_image;
      result_package.scene_name = "single_scene";
      result_package.camera_direction = params.camera_direction;  // 添加相机方向参数

      // 创建着色点云
      if (scene_data.point_cloud && !scene_data.point_cloud->empty()) {
        result_package.colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

        // 使用common_lib.h中的投影函数创建着色点云
        Eigen::Matrix4f transform_f = calib_result.transformation.cast<float>();
        ProjectPointCloudToImage(scene_data.point_cloud, transform_f, qr_detector_->GetCameraMatrix(),
                                 qr_detector_->GetDistortionCoeffs(), scene_data.image, result_package.colored_cloud);
      }

      if (!result_exporter_->ExportSingleSceneResult(result_package, config_manager_->GetParams())) {
        std::cerr << "Warning: Failed to export some results" << std::endl;
      }

      // 导出中间结果
      if (params.save_intermediate_results) {
        result_exporter_->ExportLidarProcessingResult(lidar_result, "single_scene");
        result_exporter_->ExportQrDetectionResult(qr_result, "single_scene");
      }

      // 自动记录标定数据到circle_center_record.txt文件（用于后续多场景标定）
      std::string record_filename = "circle_center_record.txt";
      if (!calib_core_->SaveCalibrationData(lidar_result.circle_centers, qr_result.circle_centers, params.output_path,
                                            record_filename)) {
        if (!options.quiet) {
          std::cout << "Warning: Failed to save calibration data to record file" << std::endl;
        }
      } else {
        if (!options.quiet) {
          std::cout << "Calibration data saved to: " << params.output_path << "/" << record_filename << std::endl;
        }
      }

      return 0;
    } catch (const std::exception& e) {
      std::cerr << "Single scene calibration error: " << e.what() << std::endl;
      return 1;
    }
  }

  /**
   * @brief 运行多场景标定
   */
  int RunMultiSceneCalibration(const CliOptions& options) {
    if (!options.quiet) {
      std::cout << "Running multi-scene calibration..." << std::endl;
    }

    try {
      const Params& params = config_manager_->GetParams();

      // 综合输入验证（命令行参数 + 配置参数）
      bool hasValidInput = false;
      std::string inputSource = "";

      // 检查记录文件输入
      if (!options.record_file.empty()) {
        hasValidInput = true;
        inputSource = "record file (--record-file)";
      }
      // 检查数据目录输入
      else if (!options.data_directory.empty()) {
        hasValidInput = true;
        inputSource = "data directory (--data-dir)";
      }
      // 检查数据集目录输入（来自配置文件）
      else if (!params.dataset_dir.empty()) {
        hasValidInput = true;
        inputSource = "dataset directory (config: dataset_dir)";
      }

      if (!hasValidInput) {
        std::cerr << "Error: No valid input source specified for multi-scene mode.\n";
        std::cerr << "Please provide one of the following:\n";
        std::cerr << "  - Record file: --record-file\n";
        std::cerr << "  - Data directory: --data-dir\n";
        std::cerr << "  - Dataset directory: set 'dataset_dir' in config file\n";
        return 1;
      }

      if (!options.quiet) {
        std::cout << "Using input source: " << inputSource << std::endl;
      }

      std::vector<CalibrationBlock> calib_blocks;

      if (!options.record_file.empty()) {
        // 检查是否需要交互式选择数据集
        std::vector<int> selected_indices = options.selected_scenes;

        if (selected_indices.empty()) {
          if (!options.quiet) {
            std::cout << "No scene selection provided. Analyzing available datasets..." << std::endl;
          }

          // 先读取记录文件以计算可用数据集数量
          std::ifstream fin(options.record_file);
          if (fin.is_open()) {
            std::vector<std::string> lines;
            std::string line;
            while (std::getline(fin, line)) {
              if (!line.empty()) lines.push_back(line);
            }
            fin.close();

            // 计算可用数据集数量
            int available_blocks = 0;
            for (size_t i = 0; i + 2 < lines.size(); ++i) {
              if (lines[i].rfind("time:", 0) == 0 && lines[i + 1].find("lidar_centers:") != std::string::npos &&
                  lines[i + 2].find("qr_centers:") != std::string::npos) {
                available_blocks++;
                i += 2;
              }
            }

            if (available_blocks < kMinScenesForMultiCalibration) {
              std::cerr << "Error: Only " << available_blocks << " datasets available, need at least "
                        << kMinScenesForMultiCalibration << " for multi-scene calibration." << std::endl;
              return 1;
            }

            std::cout << "Available datasets: " << available_blocks << " (indices: 0 to " << (available_blocks - 1)
                      << ")" << std::endl;
            std::cout << "Please specify which datasets to use (e.g., '0,1,2' or '0-2' or 'all' for all datasets): ";
            std::string input;
            std::getline(std::cin, input);

            // 处理特殊输入 "all"
            if (input == "all") {
              for (int i = 0; i < available_blocks; ++i) {
                selected_indices.push_back(i);
              }
            } else {
              // 解析用户输入
              selected_indices = Cli::ParseSceneSelection(input);
            }

            if (selected_indices.empty()) {
              std::cerr << "Error: No valid datasets selected." << std::endl;
              return 1;
            }

            // 验证选择的索引范围
            for (int idx : selected_indices) {
              if (idx < 0 || idx >= available_blocks) {
                std::cerr << "Error: Invalid index " << idx << ". Available indices: 0 to " << (available_blocks - 1)
                          << std::endl;
                return 1;
              }
            }

            if (selected_indices.size() < static_cast<size_t>(kMinScenesForMultiCalibration)) {
              std::cerr << "Error: Need at least " << kMinScenesForMultiCalibration
                        << " selected datasets for calibration (got " << selected_indices.size() << ")." << std::endl;
              return 1;
            }

            if (!options.quiet) {
              std::cout << "Selected " << selected_indices.size() << " datasets for multi-scene calibration."
                        << std::endl;
              std::cout << "Selected dataset indices: ";
              for (size_t i = 0; i < selected_indices.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << selected_indices[i];
              }
              std::cout << std::endl;
            }
          } else {
            std::cerr << "Error: Failed to open record file: " << options.record_file << std::endl;
            return 1;
          }
        }

        // 从记录文件加载选定的数据
        calib_blocks = CalibCore::LoadCalibrationDataFromFile(options.record_file, selected_indices);
      } else if (!options.data_directory.empty()) {
        // 从目录批量处理
        auto scene_list = data_loader_->LoadBatchData(options.data_directory);

        for (size_t i = 0; i < scene_list.size(); ++i) {
          if (options.max_scenes != -1 && static_cast<int>(i) >= options.max_scenes) {
            break;
          }

          if (!options.selected_scenes.empty()) {
            bool selected = std::find(options.selected_scenes.begin(), options.selected_scenes.end(),
                                      static_cast<int>(i)) != options.selected_scenes.end();
            if (!selected) continue;
          }

          const auto& scene = scene_list[i];
          if (!scene.is_valid) continue;

          // 处理场景
          auto qr_result = qr_detector_->DetectMarkers(scene.image);

          auto lidar_result = lidar_detector_->DetectCircles(scene.point_cloud);

          if (qr_result.success && lidar_result.success) {
            CalibrationBlock block;
            block.lidar_centers = lidar_result.circle_centers;
            block.camera_centers = qr_result.circle_centers;
            block.is_valid = true;
            block.timestamp = "scene_" + std::to_string(i);
            calib_blocks.push_back(block);

            if (!options.quiet) {
              std::cout << "Processed scene " << i << " successfully" << std::endl;
            }
          } else {
            // 场景处理失败的处理逻辑
            if (!options.quiet) {
              std::cout << "Scene " << i << " failed: ";
              if (!qr_result.success) {
                std::cout << "QR detection failed (" << qr_result.error_message << ")";
              }
              if (!lidar_result.success) {
                if (!qr_result.success) std::cout << ", ";
                std::cout << "LiDAR detection failed (" << lidar_result.error_message << ")";
              }
              std::cout << std::endl;
            }

            // 如果启用可视化且配置了显示失败结果，进行诊断显示
            if (visualizer_ && params.show_failed_results) {
              if (!options.quiet) {
                std::cout << "Showing failed scene " << i << " for diagnosis..." << std::endl;
              }

              if (!qr_result.success) {
                visualizer_->ShowFailedQrDetection(qr_result, scene.image);
              } else if (!lidar_result.success) {
                visualizer_->ShowFailedLidarProcessing(lidar_result, scene.point_cloud);
              }

              if (!options.quiet) {
                std::cout << "Press any key to continue to next scene..." << std::endl;
                cv::waitKey(0);
                cv::destroyAllWindows();
              }
            }

            // 如果配置为不在部分失败时继续，则退出
            if (!params.continue_on_partial_failure) {
              std::cerr << "Scene " << i << " failed and continue_on_partial_failure is disabled. Exiting."
                        << std::endl;
              return 1;
            }
          }
        }
      } else if (!params.dataset_dir.empty()) {
        // 从数据集目录批量处理（新格式）
        if (!options.quiet) {
          std::cout << "Processing dataset from: " << params.dataset_dir << std::endl;
        }

        // 使用数据集加载器处理多场景
        auto dataset_structure = dataset_loader_->LoadDataset();

        // TODO: 需要实现从DatasetStructure转换为SceneData的逻辑
        // 暂时使用空的场景列表，避免编译错误
        std::vector<SceneData> dataset_scenes;

        for (size_t i = 0; i < dataset_scenes.size(); ++i) {
          if (options.max_scenes != -1 && static_cast<int>(i) >= options.max_scenes) {
            break;
          }

          if (!options.selected_scenes.empty()) {
            bool selected = std::find(options.selected_scenes.begin(), options.selected_scenes.end(),
                                      static_cast<int>(i)) != options.selected_scenes.end();
            if (!selected) continue;
          }

          const auto& scene = dataset_scenes[i];
          if (!scene.is_valid) continue;

          // 处理场景
          auto qr_result = qr_detector_->DetectMarkers(scene.image);
          auto lidar_result = lidar_detector_->DetectCircles(scene.point_cloud);

          if (qr_result.success && lidar_result.success) {
            CalibrationBlock block;
            block.lidar_centers = lidar_result.circle_centers;
            block.camera_centers = qr_result.circle_centers;
            block.is_valid = true;
            block.timestamp = "dataset_scene_" + std::to_string(i);
            calib_blocks.push_back(block);

            if (!options.quiet) {
              std::cout << "Processed dataset scene " << i << " successfully" << std::endl;
            }
          } else {
            // 数据集场景处理失败的处理逻辑
            if (!options.quiet) {
              std::cout << "Dataset scene " << i << " failed: ";
              if (!qr_result.success) {
                std::cout << "QR detection failed (" << qr_result.error_message << ")";
              }
              if (!lidar_result.success) {
                if (!qr_result.success) std::cout << ", ";
                std::cout << "LiDAR detection failed (" << lidar_result.error_message << ")";
              }
              std::cout << std::endl;
            }

            // 如果启用可视化且配置了显示失败结果，进行诊断显示
            if (visualizer_ && params.show_failed_results) {
              if (!options.quiet) {
                std::cout << "Showing failed dataset scene " << i << " for diagnosis..." << std::endl;
              }

              if (!qr_result.success) {
                visualizer_->ShowFailedQrDetection(qr_result, scene.image);
              } else if (!lidar_result.success) {
                visualizer_->ShowFailedLidarProcessing(lidar_result, scene.point_cloud);
              }

              if (!options.quiet) {
                std::cout << "Press any key to continue to next scene..." << std::endl;
                cv::waitKey(0);
                cv::destroyAllWindows();
              }
            }

            // 如果配置为不在部分失败时继续，则退出
            if (!params.continue_on_partial_failure) {
              std::cerr << "Dataset scene " << i << " failed and continue_on_partial_failure is disabled. Exiting."
                        << std::endl;
              return 1;
            }
          }
        }
      }

      if (calib_blocks.empty()) {
        std::cerr << "No valid calibration data found" << std::endl;
        return 1;
      }

      if (calib_blocks.size() < kMinScenesForMultiCalibration) {
        std::cerr << "Need at least " << kMinScenesForMultiCalibration << " scenes for multi-scene calibration (found "
                  << calib_blocks.size() << ")" << std::endl;
        return 1;
      }

      // 执行多场景标定
      auto final_result = calib_core_->PerformMultiSceneCalibration(calib_blocks);

      if (!final_result.success) {
        std::cerr << "Multi-scene calibration failed: " << final_result.error_message << std::endl;
        return 1;
      }

      // 显示原始标定结果（伪外参）
      if (!options.quiet) {
        std::cout << "Multi-scene calibration successful!" << std::endl;
        std::cout << "Used " << calib_blocks.size() << " scenes" << std::endl;
        std::cout << "RMSE: " << std::fixed << std::setprecision(4) << final_result.rmse << " m" << std::endl;
        if (config_manager_->GetParams().camera_direction != 0) {
          std::cout << "Raw (pseudo) Transformation matrix (before correction):" << std::endl;
        } else {
          std::cout << "Transformation matrix:" << std::endl;
        }
        std::cout << final_result.transformation << std::endl;
      }

      // 根据 camera_direction + lidar_yaw_offset_deg 右乘修正矩阵，得到真实外参
      const int direction = config_manager_->GetParams().camera_direction;
      const double yaw_offset = config_manager_->GetParams().lidar_yaw_offset_deg;
      if (direction != 0 || yaw_offset != 0.0) {
        double total_yaw = GetCameraDirectionAngle(direction) + yaw_offset;
        if (yaw_offset != 0.0) {
          final_result.transformation = ApplyCorrectionByAngle(final_result.transformation, total_yaw);
        } else {
          final_result.transformation = ApplyCorrectionToTransform(final_result.transformation, direction);
        }
        // 同步更新旋转矩阵和平移向量，保持结构体内部一致
        final_result.rotation    = final_result.transformation.block<3, 3>(0, 0);
        final_result.translation = final_result.transformation.block<3, 1>(0, 3);
        if (!options.quiet) {
          std::cout << "Corrected Transformation matrix (camera_direction=" << direction
                    << ", lidar_yaw_offset_deg=" << yaw_offset << ", total=" << total_yaw << "deg):" << std::endl;
          std::cout << final_result.transformation << std::endl;
        }
      }

      // 准备导出数据
      CalibrationResultPackage final_package;
      final_package.transformation_result = final_result;
      final_package.scene_name = "multi_scene_final";
      final_package.camera_direction = config_manager_->GetParams().camera_direction;  // 添加相机方向参数

      std::vector<CalibrationResultPackage> individual_packages;
      for (size_t i = 0; i < calib_blocks.size(); ++i) {
        CalibrationResultPackage package;
        package.lidar_centers = calib_blocks[i].lidar_centers;
        package.camera_centers = calib_blocks[i].camera_centers;
        package.scene_name = "scene_" + std::to_string(i);
        package.timestamp = calib_blocks[i].timestamp;
        package.camera_direction = config_manager_->GetParams().camera_direction;  // 为每个数据包设置相机方向参数
        individual_packages.push_back(package);
      }

      // 导出结果
      if (!result_exporter_->ExportMultiSceneResult(individual_packages, final_package, config_manager_->GetParams())) {
        std::cerr << "Warning: Failed to export some results" << std::endl;
      }

      return 0;
    } catch (const std::exception& e) {
      std::cerr << "Multi-scene calibration error: " << e.what() << std::endl;
      return 1;
    }
  }

  /**
   * @brief 仅处理数据（不标定）
   */
  int RunProcessOnly(const CliOptions& options) {
    if (!options.quiet) {
      std::cout << "Running data processing only..." << std::endl;
    }

    // TODO: 实现数据处理逻辑
    // 该模式下仅运行检测器，不进行标定计算
    return 0;
  }

  /**
   * @brief 批处理模式
   */
  int RunBatchProcess(const CliOptions& options) {
    if (!options.quiet) {
      std::cout << "Running batch processing..." << std::endl;
    }

    // TODO: 实现批处理逻辑
    // 支持处理多个目录或文件的批量操作
    return 0;
  }

  /**
   * @brief 交互模式
   */
  int RunInteractive(const CliOptions& options) {
    if (!options.quiet) {
      std::cout << "Entering interactive mode..." << std::endl;
      std::cout << "Interactive mode is not yet implemented." << std::endl;
    }

    // TODO: 实现交互模式逻辑
    // 提供用户友好的交互式操作界面
    return 0;
  }

  /**
   * @brief 获取数据集输出目录
   * @param params 配置参数
   * @return std::string 数据集输出目录路径
   */
  std::string GetDatasetOutputDirectory(const Params& params) {
    std::string dataset_name = "unknown_dataset";

    // 从dataset_dir提取数据集名称
    if (!params.dataset_dir.empty()) {
      size_t last_slash = params.dataset_dir.find_last_of("/\\");
      if (last_slash != std::string::npos) {
        dataset_name = params.dataset_dir.substr(last_slash + 1);
      } else {
        dataset_name = params.dataset_dir;
      }
    }

    // 移除可能的路径相对前缀
    if (dataset_name.substr(0, 2) == "./") {
      dataset_name = dataset_name.substr(2);
    }

    return params.output_path + "/" + dataset_name;
  }

  /**
   * @brief 运行数据集结构处理 (Feature 001-1-pcd-pcd)
   * @return SceneData 处理后的场景数据，is_valid为false表示处理失败
   */
  SceneData RunDatasetProcessing(const CliOptions& options) {
    if (!options.quiet) {
      std::cout << "Running dataset structure processing..." << std::endl;
    }

    if (!dataset_loader_ || !pcd_merger_ || !video_extractor_) {
      std::cerr << "Dataset processing components not initialized" << std::endl;
      SceneData invalid_scene;
      invalid_scene.is_valid = false;
      return invalid_scene;
    }

    SceneData result_scene;

    int processing_result = lidar_camera_calib::g_exception_handler.SafeExecute(
        [&]() {
          // 获取配置参数
          const Params& params = config_manager_->GetParams();

          // 验证数据集结构
          auto validation_result = dataset_loader_->Validate();
          if (!validation_result.success) {
            if (!options.quiet) {
              std::cout << "Dataset validation failed:" << std::endl;
              for (const auto& error : validation_result.errors) {
                std::cout << "  ERROR: " << error << std::endl;
              }
              for (const auto& warning : validation_result.warnings) {
                std::cout << "  WARNING: " << warning << std::endl;
              }
            }
            throw lidar_camera_calib::DatasetValidationError("Dataset validation failed");
          }

          // 加载数据集结构
          auto dataset = dataset_loader_->LoadDataset();

          if (!options.quiet) {
            std::cout << "Dataset loaded successfully:" << std::endl;
            std::cout << "  " << dataset.GetSummary() << std::endl;
          }

          // 处理相机数据 - 提取图像
          // cv::Mat extracted_image;
          // std::string image_source_path;
          //
          // for (const auto& camera_dir : dataset.cameras) {
          //   if (!camera_dir.HasVideos()) {
          //     if (!options.quiet) {
          //       std::cout << "Skipping camera " << camera_dir.camera_id << " (no video files)" << std::endl;
          //     }
          //     continue;
          //   }
          //
          //   // 从视频提取帧
          //   std::string video_path = camera_dir.GetPrimaryVideo();
          //   video_extractor_->SetFrameSelectionStrategy(
          //       lidar_camera_calib::FrameSelectionStrategy::SPECIFIC_INDEX);  // 设置为指定索引模式
          //   video_extractor_->SetFrameIndex(10);                      // 设置提取第10帧（避免第0帧无相机画面）
          //   auto extracted_frame = video_extractor_->ExtractFrame(video_path);
          //
          //   if (!extracted_frame.IsValid()) {
          //     lidar_camera_calib::g_exception_handler.ReportWarning("Failed to extract frame from " + video_path);
          //     continue;
          //   }
          //
          //   if (!options.quiet) {
          //     std::cout << "Extracted frame from " << camera_dir.camera_id << ": " << extracted_frame.GetSummary()
          //               << std::endl;
          //   }
          //
          //   // 收集图像数据用于构建SceneData
          //   if (extracted_image.empty()) {
          //     extracted_image = extracted_frame.image;
          //     image_source_path = video_path;
          //
          //     // 可选：保存提取的帧
          //     if (params.save_intermediate_results) {
          //       std::string dataset_output_dir = GetDatasetOutputDirectory(params);
          //       // 确保数据集输出目录存在
          //       if (!result_exporter_->CreateDirectory(dataset_output_dir)) {
          //         if (!options.quiet) {
          //           std::cout << "Warning: Failed to create dataset output directory: " << dataset_output_dir
          //                     << std::endl;
          //         }
          //       } else {
          //         std::string output_path = dataset_output_dir + "/" + camera_dir.camera_id + "_extracted.jpg";
          //         if (extracted_frame.SaveToFile(output_path)) {
          //           if (!options.quiet) {
          //             std::cout << "Saved extracted frame to: " << output_path << std::endl;
          //           }
          //         }
          //       }
          //     }
          //   }
          // }

          // 处理点云数据 - 合并PCD文件
          pcl::PointCloud<pcl::PointXYZI>::Ptr merged_point_cloud;
          std::string pointcloud_source_path;

          if (dataset.lidar.HasFiles()) {
            auto merged_cloud = pcd_merger_->MergeDirectory(dataset.lidar.directory_path, params.max_pcds);

            if (!merged_cloud.IsEmpty()) {
              if (!options.quiet) {
                std::cout << "PCD files merged successfully:" << std::endl;
                std::cout << "  " << merged_cloud.GetMergeSummary() << std::endl;
              }

              pointcloud_source_path = dataset.lidar.directory_path;
              merged_point_cloud = merged_cloud.cloud;

              // 可选：保存合并的点云
              if (params.save_intermediate_results) {
                std::string output_path = params.output_path + "/merged_pointcloud.pcd";
                pcl::io::savePCDFileBinary(output_path, *merged_point_cloud);
                if (!options.quiet) {
                  std::cout << "Merged point cloud would be saved to: " << output_path << std::endl;
                }
              }
            }
          } else {
            lidar_camera_calib::g_exception_handler.ReportWarning("No PCD files found for merging");
          }

          // 构建SceneData
          result_scene.scene_name = "dataset_" + params.camera_id;
          result_scene.point_cloud = merged_point_cloud;
          result_scene.pointcloud_path = pointcloud_source_path;
          result_scene.is_valid = (merged_point_cloud && !merged_point_cloud->empty());

          if (!options.quiet) {
            std::cout << "Dataset processing completed successfully!" << std::endl;
            if (result_scene.is_valid) {
              std::cout << "SceneData created: "
                        << ", pointcloud="
                        << (merged_point_cloud ? std::to_string(merged_point_cloud->size()) + " points" : "null")
                        << std::endl;
            }
          }
        },
        "Dataset Processing");

    if (processing_result == 0) {
      return result_scene;
    } else {
      // 处理失败，返回无效的SceneData
      SceneData invalid_scene;
      invalid_scene.is_valid = false;
      return invalid_scene;
    }
  }

  int RunUndistortionPreprocessing(UndistortionResult& out_result) const {
    try {
      const Params& params = config_manager_->GetParams();  // 获取原始参数

      UndistortionConfig config;
      config.input_dataset_dir = params.dataset_dir;
      config.target_camera_id = params.camera_id;
      config.camera_model_path = params.camera_model_path;

      config.undistorted_intrinsics =
          undistortion_preprocessor_->ComputeUndistortedIntrinsics(config.camera_model_path, config.image_size);

      out_result.undistorted_intrinsics = config.undistorted_intrinsics;
      out_result.camera_model_path = config.camera_model_path;

      if (config.undistorted_intrinsics.empty()) {
        std::cerr << "Error: Failed to compute new camera intrinsics. "
                  << "Last error: " << undistortion_preprocessor_->GetLastError() << std::endl;
        return 1;
      }

      if (!undistortion_preprocessor_->ProcessDataset(config, out_result)) {
        std::cerr << "Undistortion preprocessing failed: " << out_result.error_message << std::endl;
        return 1;
      }

      // 保存图片
      std::string output_path = params.output_path + "/" + params.camera_id + "_extracted_image.jpg";
      cv::imwrite(output_path, out_result.image);
      std::cout << "saved undistorted image: " << output_path << std::endl;
      output_path = params.output_path + "/" + params.camera_id + "_undistorted_image.jpg";
      cv::imwrite(output_path, out_result.undistort_image);
      std::cout << "saved undistorted image: " << output_path << std::endl;
      out_result.success = true;
      return 0;

    } catch (const std::exception& e) {
      std::cerr << "Undistortion preprocessing error: " << e.what() << std::endl;
      return 1;
    }
  }

 private:
  // 核心组件
  std::shared_ptr<Cli> cli_;                       ///< 命令行接口组件
  std::shared_ptr<ConfigManager> config_manager_;  ///< 配置管理组件
  std::shared_ptr<DataLoader> data_loader_;        ///< 数据加载组件

  // 检测组件
  std::shared_ptr<QRDetector> qr_detector_;        ///< ArUco 标记检测器
  std::shared_ptr<LidarDetector> lidar_detector_;  ///< LiDAR 圆形目标检测器

  // 标定和输出组件
  std::shared_ptr<CalibCore> calib_core_;            ///< 标定算法核心
  std::shared_ptr<Visualizer> visualizer_;           ///< 可视化组件（可选）
  std::shared_ptr<ResultExporter> result_exporter_;  ///< 结果导出组件

  // 新数据集处理组件 (Feature 001-1-pcd-pcd)
  std::shared_ptr<lidar_camera_calib::DatasetLoader> dataset_loader_;    ///< 数据集加载器
  std::shared_ptr<lidar_camera_calib::PcdMerger> pcd_merger_;            ///< PCD文件合并器
  std::shared_ptr<lidar_camera_calib::VideoExtractor> video_extractor_;  ///< 视频帧提取器

  // 去畸变预处理组件 (v2.2+)
  std::shared_ptr<UndistortionPreprocessor> undistortion_preprocessor_;  ///< 去畸变预处理器
};

/**
 * @brief 主程序入口点
 * @param argc [in] 命令行参数个数
 * @param argv [in] 命令行参数数组
 * @return int 程序退出状态码（0表示成功，非0表示失败）
 */
int main(int argc, char* argv[]) {
  try {
    CalibrationApp app;
    return app.Run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown fatal error occurred" << std::endl;
    return 1;
  }
}
