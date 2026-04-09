/**
 * @file undistortion_preprocessor.cpp
 * @brief 去畸变预处理模块实现
 */

#include "undistortion_preprocessor.h"
#include "common_lib.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_model/camera_models/CameraFactory.h"

#include "video_extractor.h"
#include "exception_handler.h"

UndistortionPreprocessor::UndistortionPreprocessor() : verbose_(false) {}

UndistortionPreprocessor::~UndistortionPreprocessor() = default;

bool UndistortionPreprocessor::ProcessDataset(const UndistortionConfig& config, UndistortionResult& result) const {
  if (verbose_) {
    std::cout << "[UndistortionPreprocessor] Starting dataset processing..." << std::endl;
    std::cout << "  Input dataset : " << config.input_dataset_dir << std::endl;
  }

  // 1. 验证输入目录
  if (!std::filesystem::exists(config.input_dataset_dir)) {
    result.error_message = "Input dataset directory does not exist: " + config.input_dataset_dir;
    return false;
  }

  // 2. 直接使用预计算好的新内参
  if (config.undistorted_intrinsics.empty()) {
    result.error_message = "Input undistorted camera intrinsics are empty";
    return false;
  }
  const std::vector<double> undistorted_intrinsics = config.undistorted_intrinsics;
  const std::string cameras_input_dir = config.input_dataset_dir + "/cameras";

  // 4. 处理相机数据
  if (std::filesystem::exists(cameras_input_dir)) {
    for (const auto& camera_entry : std::filesystem::directory_iterator(cameras_input_dir)) {
      if (camera_entry.is_directory()) {
        std::string camera_id = camera_entry.path().filename().string();
        std::string camera_input_sub_dir = camera_entry.path().string();
        bool is_target_camera = config.target_camera_id.empty() || config.target_camera_id == camera_id;

        if (is_target_camera) {
          if (verbose_) {
            std::cout << "[UndistortionPreprocessor] Processing target camera: " << camera_id << std::endl;
          }
          for (const auto& file_entry : std::filesystem::directory_iterator(camera_input_sub_dir)) {
            std::string filename = file_entry.path().filename().string();
            std::string extension = file_entry.path().extension().string();
            std::string input_file = file_entry.path().string();

            if (extension == ".mp4" || extension == ".avi" || extension == ".mov") {
              if (!GetImageFromVideo(input_file, config, result)) {
                std::cout << "Failed to extract image from video: " << input_file << std::endl;
                return false;
              }
              return true;
            }
            std::cout << "Skipping file: " << filename << std::endl;
          }
        }
      }
    }
  }

  return false;
}

bool UndistortionPreprocessor::GetImageFromVideo(const std::string& input_video_path, const UndistortionConfig& config,
                                                 UndistortionResult& result) const {
  const auto video_extractor_ptr = std::make_shared<lidar_camera_calib::VideoExtractor>();
  video_extractor_ptr->SetOutputFormat("jpg");
  video_extractor_ptr->SetQualityFactor(95);

  video_extractor_ptr->SetFrameSelectionStrategy(
      lidar_camera_calib::FrameSelectionStrategy::SPECIFIC_INDEX);  // 设置为指定索引模式
  video_extractor_ptr->SetFrameIndex(150);                  // 设置提取第150帧（避免第0帧无相机画面）
  auto extracted_frame = video_extractor_ptr->ExtractFrame(input_video_path);

  if (!extracted_frame.IsValid()) {
    lidar_camera_calib::g_exception_handler.ReportWarning("Failed to extract frame from " + input_video_path);
    return false;
  }

  std::cout << "Extracted frame from " << input_video_path << ": " << extracted_frame.GetSummary() << std::endl;
  result.image = extracted_frame.image.clone();

  if (!ProcessImage(extracted_frame.image, result.undistort_image, config)) {
    return false;
  }
  return true;
}

bool UndistortionPreprocessor::ProcessImage(const cv::Mat& input_image, cv::Mat& output_image,
                                            const UndistortionConfig& config) const {
  if (input_image.empty()) {
    last_error_ = "Input image is empty";
    return false;
  }
  // 从配置文件创建相机
  const camera_model::CameraPtr camera_ptr =
      camera_model::CameraFactory::generateCameraFromYamlFile(config.camera_model_path);

  // 创建去畸变映射
  cv::Mat map1, map2;
  // 旋转矩阵，这里使用单位矩阵
  cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F);
  camera_ptr->initUndistortRectifyMap(map1, map2, -1, -1, input_image.size(), -1, -1, rmat);

  // 应用去畸变映射
  cv::remap(input_image, output_image, map1, map2, cv::INTER_LINEAR);
  return true;
}

std::vector<double> UndistortionPreprocessor::ComputeUndistortedIntrinsics(const std::string& camera_model_path,
                                                                           cv::Size& image_size) const {
  // 从配置文件创建相机
  const camera_model::CameraPtr camera_ptr = camera_model::CameraFactory::generateCameraFromYamlFile(camera_model_path);

  std::cout << camera_ptr->parametersToString() << std::endl;

  std::vector<double> parameters;
  camera_ptr->writeParameters(parameters);
  image_size.width = camera_ptr->imageWidth();
  image_size.height = camera_ptr->imageHeight();

  try {
    // 使用OpenCV的getOptimalNewCameraMatrix方法计算最佳去畸变后的内参矩阵
    // 首先创建畸变系数矩阵
    cv::Mat distortion_coeffs;

    // 根据相机模型类型提取参数
    switch (camera_ptr->modelType()) {
      case camera_model::Camera::KANNALA_BRANDT: {
        // KANNALA_BRANDT模型通常有8个内部参数 + 4个畸变参数
        if (parameters.size() >= 12) {
          // 取得内部参数（fx, fy, u0, v0, k2, k3, k4, k5）
          double fx = parameters[4];
          double fy = parameters[5];
          double u0 = parameters[6];
          double v0 = parameters[7];

          // 取得畸变系数
          distortion_coeffs = (cv::Mat_<double>(4, 1) << parameters[8], parameters[9], parameters[10], parameters[11]);

          // 创建原始相机矩阵
          cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);

          // 计算最优去畸变相机矩阵，alpha=0表示消除所有黑边
          cv::Mat optimal_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, image_size, 0, image_size);

          // 将矩阵转换为内参向量
          std::vector<double> undistorted_intrinsics = {
              optimal_matrix.at<double>(0, 0),  // fx
              optimal_matrix.at<double>(1, 1),  // fy
              optimal_matrix.at<double>(0, 2),  // cx
              optimal_matrix.at<double>(1, 2)   // cy
          };

          if (verbose_) {
            std::cout << "[UndistortionPreprocessor] KANNALA_BRANDT computed intrinsics: ["
                      << undistorted_intrinsics[0] << ", " << undistorted_intrinsics[1] << ", "
                      << undistorted_intrinsics[2] << ", " << undistorted_intrinsics[3] << "]" << std::endl;
          }

          return undistorted_intrinsics;
        }
        break;
      }
      case camera_model::Camera::MEI: {
        if (parameters.size() >= 9) {
          // MEI模型: fx, fy, u0, v0, gamma1, gamma2
          double fx = parameters[4];
          double fy = parameters[5];
          double u0 = parameters[6];
          double v0 = parameters[7];

          // 畸变系数k1, k2
          distortion_coeffs = (cv::Mat_<double>(2, 1) << parameters[8], parameters[9]);

          cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);
          cv::Mat optimal_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, image_size, 0, image_size);

          std::vector<double> undistorted_intrinsics = {
              optimal_matrix.at<double>(0, 0),  // fx
              optimal_matrix.at<double>(1, 1),  // fy
              optimal_matrix.at<double>(0, 2),  // cx
              optimal_matrix.at<double>(1, 2)   // cy
          };

          if (verbose_) {
            std::cout << "[UndistortionPreprocessor] MEI computed intrinsics: ["
                      << undistorted_intrinsics[0] << ", " << undistorted_intrinsics[1] << ", "
                      << undistorted_intrinsics[2] << ", " << undistorted_intrinsics[3] << "]" << std::endl;
          }

          return undistorted_intrinsics;
        }
        break;
      }
      case camera_model::Camera::PINHOLE:
      case camera_model::Camera::PINHOLE_FULL: {
        // 标准针孔模型：通常前4个参数是内部参数，后面是畸变系数
        double fx, fy, u0, v0;
        if (camera_ptr->modelType() == camera_model::Camera::PINHOLE_FULL) {
          fx = parameters[8];
          fy = parameters[9];
          u0 = parameters[10];
          v0 = parameters[11];
        } else {
          fx = parameters[0];
          fy = parameters[1];
          u0 = parameters[2];
          v0 = parameters[3];
        }

        // 为针孔模型计算畸变系数（5个参数：k1, k2, p1, p2, k3）
        int num_distortion_params = std::min(5, static_cast<int>(parameters.size() - 4));
        distortion_coeffs = cv::Mat::zeros(num_distortion_params, 1, CV_64F);
        for (int i = 0; i < num_distortion_params && i + 4 < parameters.size(); ++i) {
          distortion_coeffs.at<double>(i, 0) = parameters[4 + i];
        }

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);
        cv::Mat optimal_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, image_size, 0, image_size);

        std::vector<double> undistorted_intrinsics = {
            optimal_matrix.at<double>(0, 0),  // fx
            optimal_matrix.at<double>(1, 1),  // fy
            optimal_matrix.at<double>(0, 2),  // cx
            optimal_matrix.at<double>(1, 2)   // cy
        };

        if (verbose_) {
          std::cout << "[UndistortionPreprocessor] PINHOLE computed intrinsics: ["
                    << undistorted_intrinsics[0] << ", " << undistorted_intrinsics[1] << ", "
                    << undistorted_intrinsics[2] << ", " << undistorted_intrinsics[3] << "]" << std::endl;
        }

        return undistorted_intrinsics;
      }
      default:
        last_error_ = "Unsupported camera model: " + std::to_string(camera_ptr->modelType());
        return {};
    }

    // 如果上述方法都不适用，使用默认方法
    Eigen::Matrix3f K_rect;

    // 处理不同的相机模型
    switch (camera_ptr->modelType()) {
      case camera_model::Camera::KANNALA_BRANDT:
        // mu mv
        K_rect << parameters.at(4), 0, image_size.width / 2, 0, parameters.at(5), image_size.height / 2, 0, 0, 1;
        break;
      case camera_model::Camera::MEI:
        // gamma1 gamma2
        K_rect << parameters.at(5), 0, image_size.width / 2, 0, parameters.at(6), image_size.height / 2, 0, 0, 1;
        break;
      case camera_model::Camera::PINHOLE:
        // fx fy
        K_rect << parameters.at(4), 0, image_size.width / 2, 0, parameters.at(5), image_size.height / 2, 0, 0, 1;
        break;
      case camera_model::Camera::PINHOLE_FULL:
        // fx fy
        K_rect << parameters.at(8), 0, image_size.width / 2, 0, parameters.at(9), image_size.height / 2, 0, 0, 1;
        break;
      default:
        last_error_ = "Unsupported camera model: " + std::to_string(camera_ptr->modelType());
        return {};
    }

    // 将新的内参矩阵转换为向量格式
    std::vector<double> undistorted_intrinsics = {
        K_rect(0, 0),  // fu
        K_rect(1, 1),  // fv
        K_rect(0, 2),  // pu
        K_rect(1, 2)   // pv
    };

    if (verbose_) {
      std::cout << "[UndistortionPreprocessor] New intrinsics: [" << undistorted_intrinsics[0] << ", "
                << undistorted_intrinsics[1] << ", " << undistorted_intrinsics[2] << ", " << undistorted_intrinsics[3]
                << "]" << std::endl;
    }

    return undistorted_intrinsics;

  } catch (const cv::Exception& e) {
    last_error_ = "OpenCV error computing undistorted intrinsics: " + std::string(e.what());
    return {};
  } catch (const std::exception& e) {
    last_error_ = "Failed to compute undistorted intrinsics: " + std::string(e.what());
    return {};
  }
}


void UndistortionPreprocessor::SetVerbose(bool verbose) { verbose_ = verbose; }

std::string UndistortionPreprocessor::GetLastError() const { return last_error_; }

bool UndistortionPreprocessor::GenerateUndistortionVisualization(const std::string& input_video_path,
                                                                 const UndistortionConfig& config,
                                                                 const std::string& output_image_path,
                                                                 int frame_index) {
  try {
    // 1. 打开输入视频
    cv::VideoCapture cap(input_video_path);
    if (!cap.isOpened()) {
      last_error_ = "Cannot open input video: " + input_video_path;
      return false;
    }

    // 2. 获取视频信息
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    if (total_frames <= 0) {
      last_error_ = "Invalid video or cannot get frame count";
      return false;
    }

    // 3. 确定提取帧的索引（默认为中间帧）
    int target_frame = (frame_index >= 0) ? frame_index : total_frames / 2;
    target_frame = std::min(target_frame, total_frames - 1);

    if (verbose_) {
      std::cout << "[Visualization] Extracting frame " << target_frame << " from " << total_frames << " total frames"
                << std::endl;
    }

    // 4. 跳转到目标帧
    cap.set(cv::CAP_PROP_POS_FRAMES, target_frame);

    // 5. 读取帧
    cv::Mat original_frame;
    if (!cap.read(original_frame) || original_frame.empty()) {
      last_error_ = "Failed to read frame from video";
      return false;
    }
    cap.release();

    // 6. 执行去畸变
    cv::Mat undistorted_frame;
    if (!ProcessImage(original_frame, undistorted_frame, config)) {
      last_error_ = "Failed to undistort frame";
      return false;
    }

    // 7. 保存对比图像
    return SaveUndistortionComparison(original_frame, undistorted_frame, config.undistorted_intrinsics,
                                      output_image_path);

  } catch (const std::exception& e) {
    last_error_ = "Visualization generation failed: " + std::string(e.what());
    return false;
  }
}

bool UndistortionPreprocessor::SaveUndistortionComparison(const cv::Mat& original_image,
                                                          const cv::Mat& undistorted_image,
                                                          const std::vector<double>& undistorted_intrinsics,
                                                          const std::string& output_path) {
  try {
    // 1. 计算输出图像尺寸（缩放以适合显示）
    int display_width = 640;  // 每个图像的显示宽度
    int display_height =
        static_cast<int>(display_width * static_cast<double>(original_image.rows) / original_image.cols);

    // 2. 缩放图像
    cv::Mat original_resized, undistorted_resized;
    cv::resize(original_image, original_resized, cv::Size(display_width, display_height));
    cv::resize(undistorted_image, undistorted_resized, cv::Size(display_width, display_height));

    // 3. 创建对比图像（水平拼接）
    cv::Mat comparison_image;
    cv::hconcat(original_resized, undistorted_resized, comparison_image);

    // 4. 添加文字标签和参数信息（增强版）
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale_title = 1.0;  // 增大标题字体
    double font_scale_info = 0.6;   // 信息字体
    int thickness_title = 2;
    int thickness_info = 1;
    cv::Scalar text_color(0, 255, 0);      // 绿色
    cv::Scalar info_color(255, 255, 255);  // 白色信息文字
    cv::Scalar bg_color(0, 0, 0);          // 黑色背景

    // 左侧图像标题
    std::string left_title = "Original (Fisheye)";
    cv::putText(comparison_image, left_title, cv::Point(10, 30), font_face, font_scale_title, text_color,
                thickness_title);

    // 右侧图像标题
    std::string right_title = "Undistorted";
    cv::putText(comparison_image, right_title, cv::Point(display_width + 10, 30), font_face, font_scale_title,
                text_color, thickness_title);

    // 5. 添加详细内参信息（类似Python示例输出）
    int info_start_y = display_height - 140;
    int line_height = 20;
    int current_y = info_start_y;

    // 左侧：原始相机内参
    cv::putText(comparison_image, "Original Camera Matrix:", cv::Point(10, current_y), font_face, font_scale_info,
                info_color, thickness_info);
    current_y += line_height;

    // 右侧：新相机内参
    current_y = info_start_y;
    cv::putText(comparison_image, "New Camera Matrix:", cv::Point(display_width + 10, current_y), font_face,
                font_scale_info, info_color, thickness_info);
    current_y += line_height;

    std::stringstream fx_new;
    fx_new << std::fixed << std::setprecision(2) << "  fx' = " << undistorted_intrinsics[0];
    cv::putText(comparison_image, fx_new.str(), cv::Point(display_width + 10, current_y), font_face, font_scale_info,
                info_color, thickness_info);
    current_y += line_height;

    std::stringstream fy_new;
    fy_new << std::fixed << std::setprecision(2) << "  fy' = " << undistorted_intrinsics[1];
    cv::putText(comparison_image, fy_new.str(), cv::Point(display_width + 10, current_y), font_face, font_scale_info,
                info_color, thickness_info);
    current_y += line_height;

    std::stringstream cx_new;
    cx_new << std::fixed << std::setprecision(2) << "  cx' = " << undistorted_intrinsics[2];
    cv::putText(comparison_image, cx_new.str(), cv::Point(display_width + 10, current_y), font_face, font_scale_info,
                info_color, thickness_info);
    current_y += line_height;

    std::stringstream cy_new;
    cy_new << std::fixed << std::setprecision(2) << "  cy' = " << undistorted_intrinsics[3];
    cv::putText(comparison_image, cy_new.str(), cv::Point(display_width + 10, current_y), font_face, font_scale_info,
                info_color, thickness_info);

    // 6. 添加更明显的网格线帮助判断去畸变效果
    cv::Scalar grid_color(128, 128, 128);       // 灰色网格，增强显示
    cv::Scalar center_line_color(0, 255, 255);  // 黄色中心线
    int grid_spacing = 60;                      // 增大网格间距

    // 在原始图像上画网格
    for (int x = grid_spacing; x < display_width; x += grid_spacing) {
      cv::line(comparison_image, cv::Point(x, 0), cv::Point(x, display_height), grid_color, 1);
    }
    for (int y = grid_spacing; y < display_height; y += grid_spacing) {
      cv::line(comparison_image, cv::Point(0, y), cv::Point(display_width, y), grid_color, 1);
    }
    // 中心线
    cv::line(comparison_image, cv::Point(display_width / 2, 0), cv::Point(display_width / 2, display_height),
             center_line_color, 2);
    cv::line(comparison_image, cv::Point(0, display_height / 2), cv::Point(display_width, display_height / 2),
             center_line_color, 2);

    // 在去畸变图像上画网格
    for (int x = display_width + grid_spacing; x < 2 * display_width; x += grid_spacing) {
      cv::line(comparison_image, cv::Point(x, 0), cv::Point(x, display_height), grid_color, 1);
    }
    for (int y = grid_spacing; y < display_height; y += grid_spacing) {
      cv::line(comparison_image, cv::Point(display_width, y), cv::Point(2 * display_width, y), grid_color, 1);
    }
    // 中心线
    cv::line(comparison_image, cv::Point(display_width + display_width / 2, 0),
             cv::Point(display_width + display_width / 2, display_height), center_line_color, 2);
    cv::line(comparison_image, cv::Point(display_width, display_height / 2),
             cv::Point(2 * display_width, display_height / 2), center_line_color, 2);

    // 添加分割线
    cv::line(comparison_image, cv::Point(display_width, 0), cv::Point(display_width, display_height),
             cv::Scalar(255, 255, 255), 2);

    // 7. 保存结果图像
    if (!cv::imwrite(output_path, comparison_image)) {
      last_error_ = "Failed to save comparison image to: " + output_path;
      return false;
    }

    if (verbose_) {
      std::cout << "[Visualization] Comparison image saved to: " << output_path << std::endl;
    }

    return true;

  } catch (const std::exception& e) {
    last_error_ = "Failed to save undistortion comparison: " + std::string(e.what());
    return false;
  }
}
