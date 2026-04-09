/**
 * @file logger.h
 * @brief 独立的日志系统模块
 * @author LiDAR-Camera-Calib
 * @date 2025
 * @version 1.0
 *
 * 文件和控制台输出、线程安全操作、异步日志等功能。
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE' file,
 * which is included as part of this source code package.
 */

#pragma once

#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief 日志级别枚举
 *
 * 定义不同严重程度的日志级别，级别数值越高表示越严重。
 */
enum class LogLevel {
  kDebug = 0,    ///< 调试信息 - 详细的调试输出
  kInfo = 1,     ///< 一般信息 - 程序运行状态
  kWarning = 2,  ///< 警告信息 - 潜在问题提示
  kError = 3,    ///< 错误信息 - 程序错误但可继续
  kFatal = 4     ///< 致命错误 - 程序无法继续运行
};

/**
 * @brief 日志输出目标枚举
 *
 * 定义日志消息的输出目标，可以是控制台、文件或两者。
 */
enum class LogTarget {
  kConsoleOnly = 0,  ///< 仅控制台输出
  kFileOnly = 1,     ///< 仅文件输出
  kBoth = 2          ///< 控制台和文件都输出
};

/**
 * @brief 日志配置结构体
 *
 * 包含日志系统的完整配置参数，用于控制日志的输出行为、
 * 文件管理和性能选项。
 */
struct LogConfig {
  LogLevel console_level;     ///< 控制台日志级别
  LogLevel file_level;        ///< 文件日志级别
  LogTarget target;           ///< 输出目标
  std::string log_directory;  ///< 日志文件目录
  std::string log_filename;   ///< 日志文件名（为空则自动生成）
  size_t max_file_size;       ///< 最大文件大小（字节）
  int max_files;              ///< 最大文件数量
  bool enable_colors;         ///< 是否启用颜色输出
  bool enable_timestamps;     ///< 是否启用时间戳
  bool async_logging;         ///< 是否启用异步日志
  size_t buffer_size;         ///< 异步缓冲区大小

  LogConfig()
      : console_level(LogLevel::kInfo),
        file_level(LogLevel::kDebug),
        target(LogTarget::kConsoleOnly),
        log_directory("./logs"),
        log_filename(""),
        max_file_size(10 * 1024 * 1024),
        max_files(5),
        enable_colors(true),
        enable_timestamps(true),
        async_logging(false),
        buffer_size(1024) {}
};

/**
 * @brief 日志条目结构体
 *
 * 包含单条日志消息的完整信息，包括级别、内容、时间戳、
 * 源位置等元数据。
 */
struct LogEntry {
  LogLevel level;                                   ///< 日志级别
  std::string message;                              ///< 日志消息
  std::string file;                                 ///< 源文件名
  int line;                                         ///< 源代码行号
  std::string function;                             ///< 函数名
  std::chrono::system_clock::time_point timestamp;  ///< 时间戳
  std::thread::id thread_id;                        ///< 线程ID

  LogEntry()
      : level(LogLevel::kInfo),
        line(0),
        timestamp(std::chrono::system_clock::now()),
        thread_id(std::this_thread::get_id()) {}
};

/**
 * @class Logger
 * @brief 线程安全的独立日志系统
 *
 * 该类提供完整的日志功能来替代ROS日志系统，支持：
 * - 多级别日志记录 (DEBUG, INFO, WARNING, ERROR, FATAL)
 * - 控制台和文件输出
 * - 线程安全操作
 * - 日志文件轮转
 * - 异步日志支持
 * - 颜色输出
 * - 自定义处理器
 * - 统计信息收集
 *
 * @note 线程安全：所有公共方法都是线程安全的
 * @see LogConfig, LogEntry, LogLevel
 *
 * 使用单例模式，通过getInstance()获取唯一实例。
 */
class Logger {
 public:
  /**
   * @brief 获取单例实例
   *
   * @return Logger& 日志系统的唯一实例引用
   */
  static Logger& GetInstance();

  /**
   * @brief 析构函数
   *
   * 自动调用shutdown()确保资源正确释放。
   */
  ~Logger();

  /**
   * @brief 初始化日志系统
   *
   * @param config 日志配置参数
   * @return bool 初始化是否成功
   * @note 必须在使用日志功能前调用，重复调用将返回true
   */
  bool Initialize(const LogConfig& config = LogConfig());

  /**
   * @brief 设置日志配置
   *
   * @param config 新的日志配置参数
   */
  void SetConfig(const LogConfig& config);

  /**
   * @brief 获取当前日志配置
   *
   * @return LogConfig 当前配置的副本
   */
  LogConfig GetConfig() const;

  /**
   * @brief 记录日志消息
   *
   * @param level 日志级别
   * @param message 日志消息内容
   * @param file 源文件名（可选）
   * @param line 源代码行号（可选）
   * @param function 函数名（可选）
   */
  void Log(LogLevel level, const std::string& message, const std::string& file = "", int line = 0,
           const std::string& function = "");

  /**
   * @brief 记录调试信息
   *
   * @param message 调试消息内容
   * @param file 源文件名（可选）
   * @param line 源代码行号（可选）
   * @param function 函数名（可选）
   */
  void Debug(const std::string& message, const std::string& file = "", int line = 0, const std::string& function = "");

  /**
   * @brief 记录一般信息
   *
   * @param message 信息消息内容
   * @param file 源文件名（可选）
   * @param line 源代码行号（可选）
   * @param function 函数名（可选）
   */
  void Info(const std::string& message, const std::string& file = "", int line = 0, const std::string& function = "");

  /**
   * @brief 记录警告信息
   *
   * @param message 警告消息内容
   * @param file 源文件名（可选）
   * @param line 源代码行号（可选）
   * @param function 函数名（可选）
   */
  void Warning(const std::string& message, const std::string& file = "", int line = 0,
               const std::string& function = "");

  /**
   * @brief 记录错误信息
   *
   * @param message 错误消息内容
   * @param file 源文件名（可选）
   * @param line 源代码行号（可选）
   * @param function 函数名（可选）
   */
  void Error(const std::string& message, const std::string& file = "", int line = 0, const std::string& function = "");

  /**
   * @brief 记录致命错误信息
   *
   * @param message 致命错误消息内容
   * @param file 源文件名（可选）
   * @param line 源代码行号（可选）
   * @param function 函数名（可选）
   */
  void Fatal(const std::string& message, const std::string& file = "", int line = 0, const std::string& function = "");

  /**
   * @brief 设置控制台日志级别
   *
   * @param level 新的控制台日志级别
   */
  void SetConsoleLevel(LogLevel level);

  /**
   * @brief 设置文件日志级别
   *
   * @param level 新的文件日志级别
   */
  void SetFileLevel(LogLevel level);

  /**
   * @brief 启用/禁用文件日志
   *
   * @param enabled 是否启用文件日志
   * @param directory 日志文件目录（可选，为空则使用当前配置）
   */
  void SetFileLogging(bool enabled, const std::string& directory = "");

  /**
   * @brief 刷新日志缓冲区
   *
   * 强制将缓冲区中的日志消息写入输出目标。
   * 对于异步日志，会等待队列清空。
   */
  void Flush();

  /**
   * @brief 关闭日志系统
   *
   * 停止异步线程，关闭文件，释放所有资源。
   * 关闭后需要重新调用Initialize()才能再次使用。
   */
  void Shutdown();

  /**
   * @brief 添加自定义日志处理器
   *
   * @param handler 处理函数，接收LogEntry参数
   */
  void AddHandler(std::function<void(const LogEntry&)> handler);

  /**
   * @brief 移除所有自定义处理器
   */
  void ClearHandlers();

  /**
   * @brief 获取日志统计信息
   *
   * @return std::string 格式化的统计信息字符串
   */
  std::string GetStatistics() const;

 private:
  /**
   * @brief 私有构造函数（单例模式）
   */
  Logger();

  /**
   * @brief 禁用拷贝构造和赋值
   */
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  /**
   * @brief 处理日志条目
   *
   * @param entry 待处理的日志条目
   */
  void ProcessLogEntry(const LogEntry& entry);

  /**
   * @brief 输出到控制台
   *
   * @param entry 待输出的日志条目
   */
  void OutputToConsole(const LogEntry& entry);

  /**
   * @brief 输出到文件
   *
   * @param entry 待输出的日志条目
   */
  void OutputToFile(const LogEntry& entry);

  /**
   * @brief 格式化日志消息
   *
   * @param entry 日志条目
   * @param use_colors 是否使用颜色代码
   * @return std::string 格式化后的消息字符串
   */
  std::string FormatMessage(const LogEntry& entry, bool use_colors = false) const;

  /**
   * @brief 获取日志级别字符串
   *
   * @param level 日志级别
   * @return std::string 级别对应的字符串
   */
  std::string GetLevelString(LogLevel level) const;

  /**
   * @brief 获取日志级别颜色代码
   *
   * @param level 日志级别
   * @return std::string 级别对应的ANSI颜色代码
   */
  std::string GetLevelColor(LogLevel level) const;

  /**
   * @brief 创建日志目录
   *
   * @return bool 创建是否成功
   */
  bool CreateLogDirectory();

  /**
   * @brief 生成日志文件名
   *
   * @return std::string 生成的文件名（包含完整路径）
   */
  std::string GenerateLogFilename() const;

  /**
   * @brief 检查并轮转日志文件
   *
   * 当当前文件大小超过限制时创建新文件。
   */
  void RotateLogFile();

  /**
   * @brief 清理旧日志文件
   *
   * 删除超过max_files限制的旧文件。
   */
  void CleanupOldFiles();

  /**
   * @brief 异步日志工作线程函数
   *
   * 处理异步日志队列中的消息。
   */
  void AsyncWorker();

  /**
   * @brief 检查日志级别是否应该输出
   *
   * @param level 日志级别
   * @param target 输出目标
   * @return bool 是否应该输出
   */
  bool ShouldLog(LogLevel level, LogTarget target) const;

 private:
  LogConfig config_;                 ///< 日志配置
  mutable std::mutex config_mutex_;  ///< 配置互斥锁

  std::ofstream log_file_;         ///< 日志文件流
  mutable std::mutex file_mutex_;  ///< 文件互斥锁
  std::string current_log_file_;   ///< 当前日志文件路径

  bool initialized_;         ///< 是否已初始化
  bool shutdown_requested_;  ///< 是否请求关闭

  // 异步日志
  std::queue<LogEntry> log_queue_;              ///< 日志队列
  std::mutex queue_mutex_;                      ///< 队列互斥锁
  std::condition_variable queue_condition_;     ///< 队列条件变量
  std::unique_ptr<std::thread> worker_thread_;  ///< 工作线程

  // 自定义处理器
  std::vector<std::function<void(const LogEntry&)>> handlers_;  ///< 自定义处理器列表
  std::mutex handlers_mutex_;                                   ///< 处理器互斥锁

  // 统计信息
  mutable std::mutex stats_mutex_;                    ///< 统计互斥锁
  size_t total_messages_;                             ///< 总消息数
  size_t messages_by_level_[5];                       ///< 各级别消息数
  std::chrono::system_clock::time_point start_time_;  ///< 启动时间

  // 颜色代码
  static const std::string kColorReset;
  static const std::string kColorDebug;
  static const std::string kColorInfo;
  static const std::string kColorWarning;
  static const std::string kColorError;
  static const std::string kColorFatal;
};

// 便捷宏定义（替代ROS日志宏）
#define LIDAR_LOG_DEBUG(msg) Logger::GetInstance().Debug(msg, __FILE__, __LINE__, __FUNCTION__)

#define LIDAR_LOG_INFO(msg) Logger::GetInstance().Info(msg, __FILE__, __LINE__, __FUNCTION__)

#define LIDAR_LOG_WARNING(msg) Logger::GetInstance().Warning(msg, __FILE__, __LINE__, __FUNCTION__)

#define LIDAR_LOG_ERROR(msg) Logger::GetInstance().Error(msg, __FILE__, __LINE__, __FUNCTION__)

#define LIDAR_LOG_FATAL(msg) Logger::GetInstance().Fatal(msg, __FILE__, __LINE__, __FUNCTION__)

// 流式接口宏
#define LIDAR_LOG_STREAM(level) LoggerStream(Logger::GetInstance(), level, __FILE__, __LINE__, __FUNCTION__)

/**
 * @class LoggerStream
 * @brief 日志流辅助类
 *
 * 提供流式语法的日志记录接口，允许使用 << 操作符构建日志消息。
 * 在析构时自动调用Logger::Log()输出完整消息。
 */
class LoggerStream {
 public:
  /**
   * @brief 构造函数
   *
   * @param logger 日志器引用
   * @param level 日志级别
   * @param file 源文件名
   * @param line 行号
   * @param function 函数名
   */
  LoggerStream(Logger& logger, LogLevel level, const std::string& file, int line, const std::string& function)
      : logger_(logger), level_(level), file_(file), line_(line), function_(function) {}

  /**
   * @brief 析构函数
   *
   * 自动输出构建的日志消息。
   */
  ~LoggerStream() { logger_.Log(level_, stream_.str(), file_, line_, function_); }

  /**
   * @brief 流式输入操作符
   *
   * @tparam T 输入值类型
   * @param value 要输入的值
   * @return LoggerStream& 自身引用
   */
  template <typename T>
  LoggerStream& operator<<(const T& value) {
    stream_ << value;
    return *this;
  }

 private:
  Logger& logger_;
  LogLevel level_;
  std::string file_;
  int line_;
  std::string function_;
  std::ostringstream stream_;
};

// 流式日志宏
#define LIDAR_LOG_DEBUG_STREAM LIDAR_LOG_STREAM(LogLevel::kDebug)
#define LIDAR_LOG_INFO_STREAM LIDAR_LOG_STREAM(LogLevel::kInfo)
#define LIDAR_LOG_WARNING_STREAM LIDAR_LOG_STREAM(LogLevel::kWarning)
#define LIDAR_LOG_ERROR_STREAM LIDAR_LOG_STREAM(LogLevel::kError)
#define LIDAR_LOG_FATAL_STREAM LIDAR_LOG_STREAM(LogLevel::kFatal)

using LoggerPtr = std::shared_ptr<Logger>;