/**
 * @file cli.h
 * @brief 命令行接口模块 - 提供独立的命令行参数解析和处理功能
 * @author LiDAR-Camera-Calib ROS-Decoupling Project
 * @date 2024
 * @version 2.0.0
 *
 * 该文件实现了无ROS依赖的命令行接口，支持单场景和多场景标定模式，
 * 提供完整的参数验证、帮助信息和错误处理功能。
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>

/**
 * @brief 命令行参数结构体
 *
 * 包含了所有支持的命令行选项和配置参数，用于在程序运行时
 * 传递用户指定的各种设置和选项。
 */
struct CliOptions {
  // 输入数据
  std::string config_file;      ///< 配置文件路径
  std::string image_path;       ///< 单张图像路径
  std::string pointcloud_path;  ///< 单个点云路径
  std::string data_directory;   ///< 数据目录路径
  std::string record_file;      ///< 多场景数据记录文件


  // 输出设置
  std::string output_prefix;                ///< 输出文件前缀
  std::vector<std::string> output_formats;  ///< 输出格式列表

  // 操作模式
  enum class Mode {
    kSingleScene,   ///< 单场景标定
    kMultiScene,    ///< 多场景标定
    kProcessOnly,   ///< 仅处理（不标定）
    kBatchProcess,  ///< 批处理模式
    kInteractive    ///< 交互模式
  } mode;

  // 处理选项
  bool verbose;               ///< 详细输出
  bool quiet;                 ///< 静默模式
  bool help;                  ///< 显示帮助
  bool version;               ///< 显示版本

  // 多场景选项
  std::vector<int> selected_scenes;  ///< 选择的场景索引
  std::string scene_selection;       ///< 场景选择字符串（如"1,3,5-8"）

  // 高级选项
  bool dry_run;           ///< 空运行（不实际处理）
  int max_scenes;         ///< 最大场景数量限制
  double rmse_threshold;  ///< RMSE阈值

  // 去畸变预处理选项
  bool undistort_preprocessing;     ///< 启用去畸变预处理
  std::string distortion_model;     ///< 畸变模型类型
  double alpha;                     ///< 去畸变缩放因子
  std::string output_dataset_dir;   ///< 输出数据集目录
  bool preserve_original;           ///< 是否保留原始数据

  CliOptions()
      : mode(Mode::kSingleScene),
        verbose(false),
        quiet(false),
        help(false),
        version(false),
        dry_run(false),
        max_scenes(-1),
        rmse_threshold(-1.0),
        undistort_preprocessing(false),
        distortion_model(""),
        alpha(1.0),
        preserve_original(true) {}
};

/**
 * @class Cli
 * @brief 命令行接口类 - 提供完整的命令行参数解析和处理功能
 *
 * 主要功能：
 * - 命令行参数解析和验证
 * - 帮助信息和版本信息显示
 * - 文件和目录操作工具函数
 * - 场景选择字符串解析
 * - 错误处理和回调机制
 *
 * @note 该类是线程安全的，支持多种输出模式和格式
 * @see CliOptions
 */
class Cli {
 public:
  /**
   * @brief 构造函数 - 初始化命令行接口并注册支持的选项
   */
  Cli();

  /**
   * @brief 析构函数
   */
  ~Cli();

  /**
   * @brief 解析命令行参数
   *
   * @param argc 参数个数
   * @param argv 参数数组
   * @return CliOptions 解析后的选项结构体
   * @throw std::runtime_error 当参数解析失败时抛出异常
   */
  CliOptions ParseArguments(int argc, char* argv[]);

  /**
   * @brief 显示帮助信息
   *
   * @param program_name 程序名称
   */
  void ShowHelp(const std::string& program_name) const;

  /**
   * @brief 显示版本信息
   */
  void ShowVersion() const;

  /**
   * @brief 显示使用示例
   */
  void ShowExamples() const;

  /**
   * @brief 验证选项有效性
   *
   * @param options 选项结构体
   * @return bool 验证结果，true表示有效
   * @note 会检查文件存在性、参数冲突和数值范围等
   */
  bool ValidateOptions(const CliOptions& options);

  /**
   * @brief 显示选项摘要
   *
   * @param options 选项结构体
   */
  void ShowOptionsSummary(const CliOptions& options) const;

  /**
   * @brief 解析场景选择字符串
   *
   * @param selection_str 选择字符串（如"1,3,5-8,10"）
   * @return std::vector<int> 解析后的场景索引（去重并排序）
   * @note 支持单个数字和范围表示法，自动处理空格和无效输入
   */
  static std::vector<int> ParseSceneSelection(const std::string& selection_str);

  /**
   * @brief 检查文件是否存在
   *
   * @param file_path 文件路径
   * @return bool 文件是否存在且为常规文件
   */
  static bool FileExists(const std::string& file_path);

  /**
   * @brief 检查目录是否存在
   *
   * @param dir_path 目录路径
   * @return bool 目录是否存在
   */
  static bool DirectoryExists(const std::string& dir_path);

  /**
   * @brief 创建目录
   *
   * @param dir_path 目录路径
   * @return bool 创建是否成功
   * @note 如果目录已存在则返回true
   */
  static bool CreateDirectory(const std::string& dir_path);

  /**
   * @brief 获取文件扩展名
   *
   * @param file_path 文件路径
   * @return std::string 扩展名（不含点号）
   */
  static std::string GetFileExtension(const std::string& file_path);

  /**
   * @brief 获取文件名（不含路径和扩展名）
   *
   * @param file_path 文件路径
   * @return std::string 文件名
   */
  static std::string GetBaseName(const std::string& file_path);

  /**
   * @brief 设置错误回调函数
   *
   * @param callback 错误回调函数，参数为错误信息字符串
   * @note 如果未设置回调，错误信息将输出到stderr
   */
  void SetErrorCallback(std::function<void(const std::string&)> callback);

  /**
   * @brief 获取最后的错误信息
   *
   * @return std::string 最后发生的错误信息
   */
  std::string GetLastError() const;

 private:
  /**
   * @brief 注册命令行选项
   */
  void RegisterOptions();

  /**
   * @brief 解析单个参数
   *
   * @param arg 参数字符串
   * @param next_arg 下一个参数（可选）
   * @param options 选项结构体引用
   * @return int 消费的参数数量（1或2）
   */
  int ParseArgument(const std::string& arg, const std::string& next_arg, CliOptions& options);

  /**
   * @brief 解析长参数（--argument格式）
   *
   * @param arg 长参数字符串
   * @param next_arg 下一个参数字符串
   * @param options 选项结构体引用
   * @return int 消费的参数数量
   */
  int ParseLongArgument(const std::string& arg, const std::string& next_arg, CliOptions& options);

  /**
   * @brief 解析短参数（-a格式）
   *
   * @param arg 短参数字符串
   * @param next_arg 下一个参数字符串
   * @param options 选项结构体引用
   * @return int 消费的参数数量
   */
  int ParseShortArgument(const std::string& arg, const std::string& next_arg, CliOptions& options);

  /**
   * @brief 检查是否为有效的输出格式
   *
   * @param format 格式字符串
   * @return bool 是否为支持的格式
   */
  bool IsValidOutputFormat(const std::string& format) const;

  /**
   * @brief 规范化路径
   *
   * @param path 输入路径
   * @return std::string 规范化后的路径
   * @note 统一使用正斜杠，移除尾部斜杠
   */
  std::string NormalizePath(const std::string& path) const;

  /**
   * @brief 报告错误
   *
   * @param message 错误信息
   * @note 会调用错误回调或输出到stderr
   */
  void ReportError(const std::string& message);

  /**
   * @brief 显示彩色文本
   *
   * @param text 文本内容
   * @param color 颜色代码
   * @return std::string 带颜色的文本或原文本
   */
  std::string ColorText(const std::string& text, const std::string& color) const;

 private:
  struct ArgumentSpec {
    std::string long_name;      ///< 长参数名
    std::string short_name;     ///< 短参数名
    std::string description;    ///< 描述
    bool requires_value;        ///< 是否需要值
    std::string default_value;  ///< 默认值
  };

  std::vector<ArgumentSpec> argument_specs_;                ///< 参数规格列表
  std::map<std::string, std::string> arg_map_;              ///< 参数映射
  mutable std::string last_error_;                          ///< 最后的错误信息
  std::function<void(const std::string&)> error_callback_;  ///< 错误回调

  // 静态常量定义
  static const std::vector<std::string> kSupportedOutputFormats;  ///< 支持的输出格式列表

  // ANSI颜色代码常量
  static const std::string kColorRed;
  static const std::string kColorGreen;
  static const std::string kColorYellow;
  static const std::string kColorBlue;
  static const std::string kColorMagenta;
  static const std::string kColorCyan;
  static const std::string kColorReset;

  // 版本信息常量
  static const std::string kVersionMajor;
  static const std::string kVersionMinor;
  static const std::string kVersionPatch;
  static const std::string kBuildDate;
};

using CliPtr = std::shared_ptr<Cli>;