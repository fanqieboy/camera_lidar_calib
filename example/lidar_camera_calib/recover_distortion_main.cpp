// recover_distortion_main.cpp

#include "reverse_distortion_map.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <filesystem> // For filesystem operations
#include <vector>
#include <yaml-cpp/yaml.h> // Include yaml-cpp for YAML parsing

// 保存内参矩阵到文件，同时计算并保存FOV
void saveIntrinsicMatrix(const cv::Mat& intrinsic, const std::string& output_path, cv::Size image_size = cv::Size(0, 0)) {
    std::string intrinsic_path = output_path + "/intrinsic_matrix.yaml";
    cv::FileStorage fs(intrinsic_path, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "intrinsic_matrix" << intrinsic;

        // 如果提供了图像尺寸，计算FOV
        if (image_size.width > 0 && image_size.height > 0) {
            // 检查矩阵类型并相应处理
            cv::Mat intrinsic_double;
            if (intrinsic.type() != CV_64F) {
                intrinsic.convertTo(intrinsic_double, CV_64F);
            } else {
                intrinsic_double = intrinsic;
            }

            double fx = intrinsic_double.at<double>(0, 0);
            double fy = intrinsic_double.at<double>(1, 1);
            double cx = intrinsic_double.at<double>(0, 2);
            double cy = intrinsic_double.at<double>(1, 2);

            // 计算水平FOV
            double left_angle = atan(-cx / fx);  // 使用atan，因为是简单比例
            double right_angle = atan((image_size.width - 1 - cx) / fx);
            double hfov_rad = right_angle - left_angle;
            double hfov_deg = hfov_rad * 180.0 / M_PI;

            // 计算垂直FOV
            double top_angle = atan(-cy / fy);
            double bottom_angle = atan((image_size.height - 1 - cy) / fy);
            double vfov_rad = bottom_angle - top_angle;
            double vfov_deg = vfov_rad * 180.0 / M_PI;

            // 将FOV信息保存到YAML文件
            fs << "horizontal_fov_deg" << hfov_deg;
            fs << "vertical_fov_deg" << vfov_deg;
            fs << "horizontal_fov_rad" << hfov_rad;
            fs << "vertical_fov_rad" << vfov_rad;

            std::cout << "FOV calculated - H: " << hfov_deg << " deg, V: " << vfov_deg << " deg" << std::endl;
        }

        fs.release();
        std::cout << "Intrinsic matrix and FOV saved to " << intrinsic_path << std::endl;
    } else {
        std::cerr << "Failed to save intrinsic matrix to " << intrinsic_path << std::endl;
    }
}

// 获取文件夹中所有图片文件
std::vector<std::string> getImagePaths(const std::string& input_folder_path) {
    std::vector<std::string> image_paths;
    std::filesystem::path input_dir(input_folder_path);
    if (!std::filesystem::exists(input_dir) || !std::filesystem::is_directory(input_dir)) {
        std::cerr << "Input directory does not exist or is not a directory: " << input_folder_path << std::endl;
        return image_paths;
    }

    for (const auto& entry : std::filesystem::directory_iterator(input_dir)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            // Filter common image extensions
            if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".tiff" || ext == ".bmp") {
                image_paths.push_back(entry.path().string());
            }
        }
    }

    return image_paths;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_config_yaml>" << std::endl;
        return -1;
    }

    std::string config_yaml_path = argv[1];

    // Load configuration from YAML file
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_yaml_path);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Failed to load config file: " << config_yaml_path << std::endl;
        return -1;
    }

    // 获取配置参数
    std::string camera_yaml_path = config["camera_parameters"]["yaml_file_path"].as<std::string>();
    std::string processing_mode = config["processing_mode"].as<std::string>();
    std::string input_folder_path = config["input_output"]["input_folder"].as<std::string>();
    std::string output_folder_path = config["input_output"]["output_folder"].as<std::string>();

    // 获取去畸变参数（可选，如果没有则使用默认值）
    float fx = -1.0f, fy = -1.0f, cx = -1.0f, cy = -1.0f;
    YAML::Node undistort_params = config["undistort_params"];
    if (undistort_params) {
        if (undistort_params["fx"]) fx = undistort_params["fx"].as<float>();
        if (undistort_params["fy"]) fy = undistort_params["fy"].as<float>();
        if (undistort_params["cx"]) cx = undistort_params["cx"].as<float>();
        if (undistort_params["cy"]) cy = undistort_params["cy"].as<float>();
    }

    // 创建输出文件夹（如果不存在）
    std::filesystem::path output_dir(output_folder_path);
    if (!std::filesystem::exists(output_dir)) {
        if (!std::filesystem::create_directories(output_dir)) {
            std::cerr << "Failed to create output directory: " << output_folder_path << std::endl;
            return -1;
        }
    }

    // 加载相机参数
    camera_model::CameraConstPtr camera = loadCameraFromYaml(camera_yaml_path);
    if (!camera) {
        std::cerr << "Failed to load camera model from " << camera_yaml_path << std::endl;
        return -1;
    }
    std::cout << "Loaded camera model: " << camera->cameraName() << " (Type: "
              << static_cast<int>(camera->modelType()) << ")" << std::endl;

    // 获取输入图片列表
    std::vector<std::string> image_paths = getImagePaths(input_folder_path);
    if (image_paths.empty()) {
        std::cerr << "No image files found in input directory: " << input_folder_path << std::endl;
        return -1;
    }
    std::cout << "Found " << image_paths.size() << " image(s) to process." << std::endl;

    // 预先加载第一张图片以确定目标尺寸（假设所有图片尺寸相同）
    cv::Mat first_img = cv::imread(image_paths[0]);
    if (first_img.empty()) {
        std::cerr << "Failed to load first image from " << image_paths[0] << " for size calculation." << std::endl;
        return -1;
    }
    cv::Size image_size = first_img.size();
    std::cout << "Using image size " << image_size.width << "x" << image_size.height << " for processing." << std::endl;

    // 根据处理模式执行相应操作
    if (processing_mode == "undistort") {
        std::cout << "Performing undistortion..." << std::endl;

        // 生成去畸变映射表
        cv::Mat map1, map2;
        cv::Mat new_intrinsic = generateUndistortionMap(
            camera,
            map1, map2,
            image_size,
            fx, fy,
            cx, cy,
            cv::Mat::eye(3, 3, CV_32F)  // 使用单位矩阵作为旋转矩阵
        );

        if (new_intrinsic.empty()) {
            std::cerr << "Failed to generate undistortion map." << std::endl;
            return -1;
        }

        // 保存映射表为CSV文件
        saveMapToCSV(map1, output_folder_path + "/undistort_map1.csv");
        saveMapToCSV(map2, output_folder_path + "/undistort_map2.csv");

        // 保存新的内参矩阵
        saveIntrinsicMatrix(new_intrinsic, output_folder_path, image_size);

        // 处理每张图片
        for (const auto& img_path : image_paths) {
            std::cout << "Processing " << img_path << " ... ";

            // 加载原始畸变图像
            cv::Mat distorted_img = cv::imread(img_path);
            if (distorted_img.empty()) {
                std::cerr << "Failed to load distorted image from " << img_path << std::endl;
                continue; // Skip this file and continue with the next one
            }

            // 检查当前图片尺寸是否与用于生成map的尺寸一致
            if (distorted_img.size() != image_size) {
                std::cerr << "Image " << img_path << " has size " << distorted_img.size()
                         << ", which differs from the map size " << image_size << ". Skipping." << std::endl;
                continue; // Or handle differently if needed
            }

            // 重映射以生成去畸变图像
            cv::Mat undistorted_image;
            cv::remap(distorted_img, undistorted_image, map1, map2,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            // 保存结果到输出文件夹，添加前缀表示是去畸变图像
            std::filesystem::path img_path_obj(img_path);
            std::string output_img_name = "undistorted_" + img_path_obj.filename().string();
            std::string output_img_path = output_folder_path + "/" + output_img_name;
            bool save_success = cv::imwrite(output_img_path, undistorted_image);

            if (save_success) {
                std::cout << "Done." << std::endl;
            } else {
                std::cerr << "Failed to save undistorted image to " << output_img_path << std::endl;
            }
        }
    }
    else if (processing_mode == "distort") {
        std::cout << "Performing reverse distortion (recovering original distortion)..." << std::endl;

        // 生成反向映射表（从去畸变到原始畸变）
        cv::Mat reverse_map1, reverse_map2;
        cv::Mat K_rect = generateDistortionMapFromUndistorted(
            camera,
            reverse_map1,
            reverse_map2,
            image_size,
            fx, fy,
            cx, cy,
            cv::Mat::eye(3, 3, CV_32F)  // 使用单位矩阵作为旋转矩阵
        );

        if (K_rect.empty()) {
            std::cerr << "Failed to generate reverse map." << std::endl;
            return -1;
        }
        std::cout << "Reverse map generated successfully." << std::endl;

        // 保存反向映射表为CSV文件
        saveMapToCSV(reverse_map1, output_folder_path + "/reverse_distort_map1.csv");
        saveMapToCSV(reverse_map2, output_folder_path + "/reverse_distort_map2.csv");

        // 保存用于反向变换的内参矩阵
        saveIntrinsicMatrix(K_rect, output_folder_path, image_size);

        // 处理每张图片
        for (const auto& img_path : image_paths) {
            std::cout << "Processing " << img_path << " ... ";

            // 加载去畸变后的图像
            cv::Mat undistorted_img = cv::imread(img_path);
            if (undistorted_img.empty()) {
                std::cerr << "Failed to load undistorted image from " << img_path << std::endl;
                continue; // Skip this file and continue with the next one
            }

            // 检查当前图片尺寸是否与用于生成map的尺寸一致
            if (undistorted_img.size() != image_size) {
                std::cerr << "Image " << img_path << " has size " << undistorted_img.size()
                         << ", which differs from the map size " << image_size << ". Skipping." << std::endl;
                continue; // Or handle differently if needed
            }

            // 重映射以恢复原始畸变
            cv::Mat recovered_distorted_image;
            cv::remap(undistorted_img, recovered_distorted_image, reverse_map1, reverse_map2,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

            // 保存结果到输出文件夹，添加前缀表示是恢复的畸变图像
            std::filesystem::path img_path_obj(img_path);
            std::string output_img_name = "distorted_" + img_path_obj.filename().string();
            std::string output_img_path = output_folder_path + "/" + output_img_name;
            bool save_success = cv::imwrite(output_img_path, recovered_distorted_image);

            if (save_success) {
                std::cout << "Done." << std::endl;
            } else {
                std::cerr << "Failed to save recovered image to " << output_img_path << std::endl;
            }
        }
    }
    else {
        std::cerr << "Invalid processing mode: " << processing_mode
                  << ". Valid modes are 'undistort' and 'distort'." << std::endl;
        return -1;
    }

    std::cout << "Batch processing completed." << std::endl;
    return 0;
}