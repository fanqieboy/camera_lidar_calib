#include "reverse_distortion_map.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <opencv2/core/eigen.hpp> // Required for cv2eigen and eigen2cv
#include "camera_model/camera_models/CameraFactory.h"
#include "camera_model/camera_models/CataCamera.h"
#include "camera_model/camera_models/EquidistantCamera.h"
#include "camera_model/camera_models/FovCamera.h"
#include "camera_model/camera_models/PinholeCamera.h"
#include "camera_model/camera_models/PinholeFullCamera.h"
#include "camera_model/camera_models/PolyFisheyeCamera.h"
#include "camera_model/camera_models/ScaramuzzaCamera.h"
#include "camera_model/camera_models/SplineCamera.h"

// 从YAML文件加载相机参数
camera_model::CameraConstPtr loadCameraFromYaml(const std::string& yaml_path) {
    return camera_model::CameraFactory::instance()->generateCameraFromYamlFile(yaml_path);
}

// 生成用于去畸变的映射表
cv::Mat generateUndistortionMap(
    const camera_model::CameraConstPtr& camera_model,
    cv::Mat& map1,
    cv::Mat& map2,
    cv::Size undistorted_image_size,
    float fx,
    float fy,
    float cx,
    float cy,
    cv::Mat rmat
) {
    // 直接使用相机模型提供的 initUndistortRectifyMap 方法
    return camera_model->initUndistortRectifyMap(map1, map2, fx, fy, undistorted_image_size, cx, cy, rmat);
}

// 辅助函数，模拟EquidistantCamera::backprojectSymmetric的行为
// 它计算从图像平面坐标(u, v)通过畸变模型反推到球面坐标的角度theta和phi
// 针对等距投影相机
void undistortAndUnprojectToSphere(double u, double v, const camera_model::EquidistantCamera& cam, double& theta, double& phi) {
    const auto& params = cam.getParameters();
    double inv_mu = 1.0 / params.mu();
    double inv_mv = 1.0 / params.mv();
    double u0 = params.u0();
    double v0 = params.v0();

    // 将像素坐标转换为归一化平面坐标
    double x = inv_mu * (u - u0);
    double y = inv_mv * (v - v0);

    Eigen::Vector2d p_u(x, y);
    double p_u_norm = p_u.norm();

    if (p_u_norm < 1e-10) {
        phi = 0.0;
    } else {
        phi = atan2(p_u(1), p_u(0)); // atan2(y, x) -> phi
    }

    // 使用与 EquidistantCamera::backprojectSymmetric 相同的多项式求根方法来找 theta
    // 目标是找到 theta 使得 r(k2,k3,k4,k5,theta) * theta' = p_u_norm (其中 theta' 是经过畸变后的角度)
    // 实际上是求解 f(theta) = r(k2,k3,k4,k5,theta) - p_u_norm = 0
    // 这里复用 backprojectSymmetric 的逻辑，但传入的是归一化后的坐标范数 p_u_norm

    double tol = 1e-10;
    int npow = 9;
    if (params.k5() == 0.0) {
        npow -= 2;
    }
    if (params.k4() == 0.0) {
        npow -= 2;
    }
    if (params.k3() == 0.0) {
        npow -= 2;
    }
    if (params.k2() == 0.0) {
        npow -= 2;
    }

    Eigen::MatrixXd coeffs(npow + 1, 1);
    coeffs.setZero();
    coeffs(0) = -p_u_norm; // 常数项为 -p_u_norm
    coeffs(1) = 1.0;       // theta^1 项的系数为 1

    if (npow >= 3) {
        coeffs(3) = params.k2();
    }
    if (npow >= 5) {
        coeffs(5) = params.k3();
    }
    if (npow >= 7) {
        coeffs(7) = params.k4();
    }
    if (npow >= 9) {
        coeffs(9) = params.k5();
    }

    if (npow == 1) {
        theta = p_u_norm;
    } else {
        // 特征值求解方法，与 backprojectSymmetric 一致
        Eigen::MatrixXd A(npow, npow);
        A.setZero();
        A.block(1, 0, npow - 1, npow - 1).setIdentity();
        A.col(npow - 1) = -coeffs.block(0, 0, npow, 1) / coeffs(npow);

        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        Eigen::MatrixXcd eigval = es.eigenvalues();

        std::vector<double> thetas;
        for (int i = 0; i < eigval.rows(); ++i) {
            if (std::abs(eigval(i).imag()) > tol) {
                continue; // 跳过非实数根
            }

            double t = eigval(i).real();

            if (t < -tol) {
                continue; // 跳过负数根
            } else if (t < 0.0) {
                t = 0.0; // 非负性修正
            }

            thetas.push_back(t);
        }

        if (thetas.empty()) {
            theta = p_u_norm; // 如果没找到合适的实数根，默认使用 p_u_norm
        } else {
            theta = *std::min_element(thetas.begin(), thetas.end()); // 选择最小的正值作为 theta
        }
    }
}

cv::Mat generateDistortionMapFromUndistorted(
    const camera_model::CameraConstPtr& camera_model,
    cv::Mat& reverse_map1,
    cv::Mat& reverse_map2,
    cv::Size undistorted_image_size,
    float fx,
    float fy,
    float cx,
    float cy,
    cv::Mat rmat
) {
    // 为保持兼容性，当相机模型为等距投影模型时，使用专门的实现
    if (camera_model->modelType() == camera_model::Camera::KANNALA_BRANDT) {
        // 将基类指针转换为具体类型
        // 使用 boost::dynamic_pointer_cast 而不是 std::dynamic_pointer_cast
        boost::shared_ptr<const camera_model::EquidistantCamera> equidistant_cam =
            boost::dynamic_pointer_cast<const camera_model::EquidistantCamera>(camera_model);
        if (equidistant_cam) {
            cv::Size original_image_size(equidistant_cam->imageWidth(), equidistant_cam->imageHeight());

            cv::Mat mapX_rev = cv::Mat::zeros(original_image_size.height, original_image_size.width, CV_32F);
            cv::Mat mapY_rev = cv::Mat::zeros(original_image_size.height, original_image_size.width, CV_32F);

            Eigen::Matrix3f K_rect; // 去畸变目标图像的内参矩阵
            if (cx == -1.0f && cy == -1.0f) {
                K_rect << fx, 0, undistorted_image_size.width / 2, 0, fy, undistorted_image_size.height / 2, 0, 0, 1;
            } else {
                K_rect << fx, 0, cx, 0, fy, cy, 0, 0, 1;
            }

            if (fx == -1.0f || fy == -1.0f) {
                K_rect(0, 0) = equidistant_cam->getParameters().mu();
                K_rect(1, 1) = equidistant_cam->getParameters().mv();
            }

            Eigen::Matrix3f R; // 从原始相机坐标系到目标相机坐标系的旋转
            cv::cv2eigen(rmat, R);

            for (int v_orig = 0; v_orig < original_image_size.height; ++v_orig) {
                for (int u_orig = 0; u_orig < original_image_size.width; ++u_orig) {
                    // Step 1: 对于原始图像上的点 (u_orig, v_orig)，计算其在相机坐标系下的方向向量 ray_orig
                    double theta, phi;
                    undistortAndUnprojectToSphere(u_orig, v_orig, *equidistant_cam, theta, phi);

                    // 由 theta 和 phi 得到单位向量
                    Eigen::Vector3f ray_orig;
                    ray_orig << sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta);

                    // Step 2: 应用旋转矩阵 R 和目标内参 K_rect
                    Eigen::Vector3f ray_rect = R * ray_orig; // 经过旋转后的新方向向量

                    if (std::abs(ray_rect(2)) < 1e-10) {
                        // 避免除零错误，将无效点标记为负值
                        mapX_rev.at<float>(v_orig, u_orig) = -1;
                        mapY_rev.at<float>(v_orig, u_orig) = -1;
                        continue;
                    }

                    // 投影到去畸变图像平面
                    float u_target = (K_rect(0, 0) * ray_rect(0) + K_rect(0, 2) * ray_rect(2)) / ray_rect(2);
                    float v_target = (K_rect(1, 1) * ray_rect(1) + K_rect(1, 2) * ray_rect(2)) / ray_rect(2);

                    // Step 3: 存储映射 (u_orig, v_orig) -> (u_target, v_target)
                    mapX_rev.at<float>(v_orig, u_orig) = u_target;
                    mapY_rev.at<float>(v_orig, u_orig) = v_target;
                }
            }

            // Step 4: 转换映射格式
            cv::convertMaps(mapX_rev, mapY_rev, reverse_map1, reverse_map2, CV_32FC1, false);

            cv::Mat K_rect_cv;
            cv::eigen2cv(K_rect, K_rect_cv);
            return K_rect_cv;
        }
    }
    // 对于其他相机模型，我们实现一个通用的反向映射计算方法
    // 使用相机模型的 initUndistortRectifyMap 方法生成正向映射，然后通过插值获得逆映射
    else {
        // 首先，生成从原始畸变图像到去畸变图像的正向映射
        cv::Size original_size(camera_model->imageWidth(), camera_model->imageHeight());

        cv::Mat forward_map1, forward_map2;
        cv::Mat intrinsic_rect = camera_model->initUndistortRectifyMap(
            forward_map1, forward_map2,
            fx, fy,
            undistorted_image_size,
            cx, cy,
            rmat
        );

        if (intrinsic_rect.empty()) {
            std::cerr << "Failed to generate forward undistortion map." << std::endl;
            return cv::Mat(); // Return empty matrix to indicate error
        }

        // 然后，生成反向映射：从去畸变图像到原始畸变图像
        // 创建一个反向查找表，对去畸变图像中的每个点，找到其在原始图像中的坐标
        cv::Mat mapX_rev = cv::Mat::zeros(undistorted_image_size.height, undistorted_image_size.width, CV_32F);
        cv::Mat mapY_rev = cv::Mat::zeros(undistorted_image_size.height, undistorted_image_size.width, CV_32F);

        // 构建反向映射的查找表
        // 对每个去畸变图像中的像素 (u, v)，寻找它在原始图像中对应的位置
        for (int v_dist = 0; v_dist < undistorted_image_size.height; ++v_dist) {
            for (int u_dist = 0; u_dist < undistorted_image_size.width; ++u_dist) {
                // 通过查找正向映射表中与当前位置最匹配的点来构建反向映射
                // 这是一个逆向查找过程
                bool found = false;
                float min_dist_sq = std::numeric_limits<float>::max();
                int best_u_orig = -1, best_v_orig = -1;

                // 在原始图像中查找，找到正向映射中与当前位置(u_dist, v_dist)最接近的点
                for (int v_orig = 0; v_orig < original_size.height && !found; v_orig += 2) { // 为了效率，采样查找
                    for (int u_orig = 0; u_orig < original_size.width && !found; u_orig += 2) {
                        float map_u = forward_map1.at<float>(v_orig, u_orig);
                        float map_v = forward_map2.at<float>(v_orig, u_orig);

                        if (map_u >= 0 && map_v >= 0) { // 有效映射
                            float dx = map_u - u_dist;
                            float dy = map_v - v_dist;
                            float dist_sq = dx * dx + dy * dy;

                            if (dist_sq < min_dist_sq) {
                                min_dist_sq = dist_sq;
                                best_u_orig = u_orig;
                                best_v_orig = v_orig;

                                // 如果距离足够小，认为找到了匹配
                                if (dist_sq < 0.25f) { // 0.5像素的阈值
                                    found = true;
                                }
                            }
                        }
                    }
                }

                // 如果找到了匹配点，设置反向映射
                if (best_u_orig >= 0 && best_v_orig >= 0) {
                    mapX_rev.at<float>(v_dist, u_dist) = static_cast<float>(best_u_orig);
                    mapY_rev.at<float>(v_dist, u_dist) = static_cast<float>(best_v_orig);
                } else {
                    // 没有找到匹配点，标记为无效
                    mapX_rev.at<float>(v_dist, u_dist) = -1;
                    mapY_rev.at<float>(v_dist, u_dist) = -1;
                }
            }
        }

        // 转换映射格式
        cv::convertMaps(mapX_rev, mapY_rev, reverse_map1, reverse_map2, CV_32FC1, false);

        return intrinsic_rect;
    }
    return cv::Mat(); // Add return statement for safety
}

// 将cv::Mat映射表保存为CSV文件
void saveMapToCSV(const cv::Mat& map, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件 " << filename << " 进行写入！" << std::endl;
        return;
    }

    // 写入CSV头部信息
    file << "# 映射表数据 - CSV格式" << std::endl;
    file << "# 数据维度: " << map.rows << "x" << map.cols << ", 类型: " << map.type() << ", 通道数: " << map.channels() << std::endl;

    // 根据Mat的数据类型和通道数写入数据
    for (int i = 0; i < map.rows; ++i) {
        for (int j = 0; j < map.cols; ++j) {
            if (map.channels() == 1) {
                // 单通道数据
                if (map.depth() == CV_32F) {
                    // 单精度浮点类型
                    float val = map.at<float>(i, j);
                    if (j == 0) {
                        file << val;
                    } else {
                        file << "," << val;
                    }
                } else if (map.depth() == CV_64F) {
                    // 双精度浮点类型
                    double val = map.at<double>(i, j);
                    if (j == 0) {
                        file << val;
                    } else {
                        file << "," << val;
                    }
                } else {
                    // 其他类型转换为double
                    double val = static_cast<double>(map.at<uchar>(i, j));
                    if (j == 0) {
                        file << val;
                    } else {
                        file << "," << val;
                    }
                }
            } else if (map.channels() == 2) {
                // 两通道数据（如remap使用的映射表）
                if (map.depth() == CV_32F) {
                    cv::Vec2f val = map.at<cv::Vec2f>(i, j);
                    if (j == 0) {
                        file << val[0] << "," << val[1];
                    } else {
                        file << "," << val[0] << "," << val[1];
                    }
                } else if (map.depth() == CV_64F) {
                    cv::Vec2d val = map.at<cv::Vec2d>(i, j);
                    if (j == 0) {
                        file << val[0] << "," << val[1];
                    } else {
                        file << "," << val[0] << "," << val[1];
                    }
                } else {
                    // 其他类型按double处理
                    cv::Vec2f val = map.at<cv::Vec2f>(i, j);
                    if (j == 0) {
                        file << static_cast<double>(val[0]) << "," << static_cast<double>(val[1]);
                    } else {
                        file << "," << static_cast<double>(val[0]) << "," << static_cast<double>(val[1]);
                    }
                }
            } else {
                // 多于2个通道的情况
                std::cerr << "警告: 检测到超过2个通道的映射表，这可能不是标准的remap格式！" << std::endl;
            }
        }
        file << std::endl;
    }

    file.close();
    std::cout << "映射表已保存到: " << filename << std::endl;
}