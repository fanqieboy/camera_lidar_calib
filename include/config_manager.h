/**
 * @file config_manager.h
 * @brief 配置管理系统头文件
 *
 * 该文件定义了独立的配置管理系统，提供类型安全的参数访问接口，
 * 支持YAML配置文件加载、参数验证和默认值处理，完全替代ROS参数服务器功能。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

// 前向声明，避免直接包含 YAML 头文件
namespace YAML {
class Node;
}

// 复用现有的参数结构体
struct Params {
  double x_min, x_max, y_min, y_max, z_min, z_max;
  double fx, fy, cx, cy;
  std::vector<double> distortion_coeffs;
  double marker_size, delta_width_qr_center, delta_height_qr_center;
  double delta_width_circles, delta_height_circles, circle_radius;
  int min_detected_markers;
  std::string image_path;
  std::string pcd_path;
  std::string data_path;
  std::string lidar_topic;
  std::string output_path;
  std::string camera_model_path;

  // 新数据集支持参数 (Feature 001-1-pcd-pcd)
  std::string dataset_dir;          ///< 数据集根目录 (包含 cameras/ 和 lidar/ 目录)
  std::string camera_id;            ///< 相机ID过滤 (如 "camera_0")，空字符串表示所有相机
  int max_pcds;                     ///< 最大PCD文件合并数量，-1表示不限制

  // 可选参数 (独立版本特有)
  bool enable_visualization;        ///< 是否启用可视化
  bool save_intermediate_results;   ///< 是否保存中间结果
  bool show_failed_results;         ///< 是否在失败时显示中间结果进行诊断
  bool continue_on_partial_failure; ///< 多场景模式下是否在部分失败时继续
  std::string log_level;            ///< 日志级别: DEBUG, INFO, WARN, ERROR

  // 去畸变预处理参数
  std::string output_dataset_dir;   ///< 输出数据集目录
  bool preserve_original_data;      ///< 是否保留原始数据

  // 相机方向参数
  int camera_direction;             ///< 相机方向 (0:正前方,1:背后,2:左侧,3:右侧)
  double lidar_yaw_offset_deg;      ///< 雷达安装附加偏航角偏差(度,逆时针为正)，叠加在camera_direction之上，默认0.0
};

/**
 * @class ConfigManager
 * @brief 配置管理器类 - 提供独立的参数管理系统
 *
 * 主要功能:
 * - 从 YAML 文件中加载配置参数
 * - 提供类型安全的参数访问接口
 * - 参数验证和默认值处理
 * - 完全替代 ROS 参数服务器的功能
 *
 * @note 线程安全: 该类不保证线程安全，需要外部同步
 * @see Params 结构体定义了所有配置参数
 */
class ConfigManager {
 public:
  /**
   * @brief 构造函数
   */
  ConfigManager();

  /**
   * @brief 析构函数
   */
  ~ConfigManager();

  /**
   * @brief 从 YAML 文件加载配置
   *
   * @param config_path 配置文件路径
   * @return true 加载成功
   * @return false 加载失败
   */
  bool LoadConfig(const std::string& config_path);

  /**
   * @brief 加载默认配置
   *
   * @return true 加载成功
   * @return false 加载失败
   */
  bool LoadDefaultConfig();

  /**
   * @brief 获取Params结构体
   *
   * @return const Params& 参数结构体引用
   */
  const Params& GetParams() const;

  /**
   * @brief 模板方法：获取指定类型的参数值
   *
   * @tparam T 参数类型
   * @param key 参数键名
   * @param default_value 默认值
   * @return T 参数值
   */
  template <typename T>
  T GetParam(const std::string& key, const T& default_value) const;

  /**
   * @brief 验证配置参数的合法性
   *
   * @return true 参数合法
   * @return false 参数非法
   */
  bool ValidateParams() const;

  /**
   * @brief 打印当前配置参数
   */
  void PrintParams() const;

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 错误信息
   */
  std::string GetLastError() const;

  /**
   * @brief 检查配置是否已加载
   *
   * @return true 已加载
   * @return false 未加载
   */
  bool IsLoaded() const;

 private:
  /**
   * @brief 设置默认参数值
   */
  void SetDefaultParams();

  /**
   * @brief 从YAML节点解析参数
   *
   * @param config YAML配置节点
   * @return true 解析成功
   * @return false 解析失败
   */
  bool ParseYamlNode(const YAML::Node& config);

  /**
   * @brief 从相机模型文件加载内参和畸变系数
   *
   * @param model_file 相机模型YAML文件路径
   * @return true 加载成功
   * @return false 加载失败
   */
  bool LoadCameraModel(const std::string& model_file);

  /**
   * @brief 根据data_path自动搜索图像和点云文件
   *
   * @return true 搜索到完整的一对数据
   * @return false 搜索失败
   */
  bool SearchDataFiles();

  /**
   * @brief 验证路径参数
   *
   * @param path 路径字符串
   * @param param_name 参数名称
   * @return true 路径有效
   * @return false 路径无效
   */
  bool ValidatePath(const std::string& path, const std::string& param_name) const;

  /**
   * @brief 验证数值参数范围
   *
   * @param value 参数值
   * @param min_val 最小值
   * @param max_val 最大值
   * @param param_name 参数名称
   * @return true 数值有效
   * @return false 数值无效
   */
  bool ValidateRange(double value, double min_val, double max_val, const std::string& param_name) const;

 private:
  Params params_;                            ///< 参数结构体
  std::unique_ptr<YAML::Node> config_node_;  ///< YAML 配置节点
  mutable std::string last_error_;           ///< 最后的错误信息
  bool loaded_;                              ///< 是否已加载配置
  std::string config_file_path_;             ///< 配置文件路径
};

// 模板方法的特化声明
template <>
double ConfigManager::GetParam<double>(const std::string& key, const double& default_value) const;
template <>
int ConfigManager::GetParam<int>(const std::string& key, const int& default_value) const;
template <>
std::string ConfigManager::GetParam<std::string>(const std::string& key, const std::string& default_value) const;
template <>
bool ConfigManager::GetParam<bool>(const std::string& key, const bool& default_value) const;