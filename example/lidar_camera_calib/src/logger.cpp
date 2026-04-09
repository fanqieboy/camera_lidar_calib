

#include "logger.h"
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <ctime>

// 颜色代码定义
const std::string Logger::COLOR_RESET = "\033[0m";
const std::string Logger::COLOR_DEBUG = "\033[36m";    // 青色
const std::string Logger::COLOR_INFO = "\033[32m";     // 绿色
const std::string Logger::COLOR_WARNING = "\033[33m";  // 黄色
const std::string Logger::COLOR_ERROR = "\033[31m";    // 红色
const std::string Logger::COLOR_FATAL = "\033[35m";    // 紫色

Logger::Logger()
    : initialized_(false),
      shutdown_requested_(false),
      total_messages_(0),
      start_time_(std::chrono::system_clock::now()) {
  // 初始化统计数组
  for (int i = 0; i < 5; ++i) {
    messages_by_level_[i] = 0;
  }
}

Logger::~Logger() { Shutdown(); }

Logger& Logger::GetInstance() {
  static Logger instance;
  return instance;
}

bool Logger::Initialize(const LogConfig& config) {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  std::lock_guard<std::mutex> file_lock(file_mutex_);

  if (initialized_) {
    return true;
  }

  config_ = config;
  shutdown_requested_ = false;

  // 创建日志目录
  if (config_.target == LogTarget::kFileOnly || config_.target == LogTarget::kBoth) {
    if (!CreateLogDirectory()) {
      std::cerr << "Failed to create log directory: " << config_.log_directory << std::endl;
      return false;
    }

    // 打开日志文件
    current_log_file_ = GenerateLogFilename();
    log_file_.open(current_log_file_, std::ios::app);
    if (!log_file_.is_open()) {
      std::cerr << "Failed to open log file: " << current_log_file_ << std::endl;
      return false;
    }
  }

  // 启动异步工作线程
  if (config_.async_logging) {
    worker_thread_ = std::make_unique<std::thread>(&Logger::AsyncWorker, this);
  }

  initialized_ = true;

  // 记录启动信息
  Info("Logger initialized successfully", __FILE__, __LINE__, __FUNCTION__);

  return true;
}

void Logger::SetConfig(const LogConfig& config) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
}

LogConfig Logger::GetConfig() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return config_;
}

void Logger::Log(LogLevel level, const std::string& message, const std::string& file, int line,
                 const std::string& function) {
  if (!initialized_ || shutdown_requested_) {
    return;
  }

  // 创建日志条目
  LogEntry entry;
  entry.level = level;
  entry.message = message;
  entry.file = file;
  entry.line = line;
  entry.function = function;
  entry.timestamp = std::chrono::system_clock::now();
  entry.thread_id = std::this_thread::get_id();

  // 更新统计信息
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_messages_++;
    messages_by_level_[static_cast<int>(level)]++;
  }

  if (config_.async_logging) {
    // 异步日志
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      log_queue_.push(entry);
    }
    queue_condition_.notify_one();
  } else {
    // 同步日志
    ProcessLogEntry(entry);
  }
}

void Logger::Debug(const std::string& message, const std::string& file, int line, const std::string& function) {
  Log(LogLevel::kDebug, message, file, line, function);
}

void Logger::Info(const std::string& message, const std::string& file, int line, const std::string& function) {
  Log(LogLevel::kInfo, message, file, line, function);
}

void Logger::Warning(const std::string& message, const std::string& file, int line, const std::string& function) {
  Log(LogLevel::kWarning, message, file, line, function);
}

void Logger::Error(const std::string& message, const std::string& file, int line, const std::string& function) {
  Log(LogLevel::kError, message, file, line, function);
}

void Logger::Fatal(const std::string& message, const std::string& file, int line, const std::string& function) {
  Log(LogLevel::kFatal, message, file, line, function);
}

void Logger::SetConsoleLevel(LogLevel level) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_.console_level = level;
}

void Logger::SetFileLevel(LogLevel level) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_.file_level = level;
}

void Logger::SetFileLogging(bool enabled, const std::string& directory) {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  std::lock_guard<std::mutex> file_lock(file_mutex_);

  if (enabled) {
    if (!directory.empty()) {
      config_.log_directory = directory;
    }
    config_.target = (config_.target == LogTarget::kConsoleOnly) ? LogTarget::kBoth : LogTarget::kFileOnly;

    if (!log_file_.is_open()) {
      CreateLogDirectory();
      current_log_file_ = GenerateLogFilename();
      log_file_.open(current_log_file_, std::ios::app);
    }
  } else {
    config_.target = LogTarget::kConsoleOnly;
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }
}

void Logger::Flush() {
  if (config_.async_logging) {
    // 等待队列清空
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_condition_.wait(lock, [this] { return log_queue_.empty(); });
  }

  std::lock_guard<std::mutex> file_lock(file_mutex_);
  if (log_file_.is_open()) {
    log_file_.flush();
  }
  std::cout.flush();
  std::cerr.flush();
}

void Logger::Shutdown() {
  if (!initialized_ || shutdown_requested_) {
    return;
  }

  shutdown_requested_ = true;

  // 停止异步工作线程
  if (worker_thread_ && worker_thread_->joinable()) {
    queue_condition_.notify_all();
    worker_thread_->join();
    worker_thread_.reset();
  }

  // 关闭文件
  {
    std::lock_guard<std::mutex> lock(file_mutex_);
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

  initialized_ = false;
}

void Logger::AddHandler(std::function<void(const LogEntry&)> handler) {
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  handlers_.push_back(handler);
}

void Logger::ClearHandlers() {
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  handlers_.clear();
}

std::string Logger::GetStatistics() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);

  auto now = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);

  std::ostringstream oss;
  oss << "=== 日志统计信息 ===\n";
  oss << "运行时间: " << duration.count() << " 秒\n";
  oss << "总消息数: " << total_messages_ << "\n";
  oss << "DEBUG: " << messages_by_level_[0] << "\n";
  oss << "INFO: " << messages_by_level_[1] << "\n";
  oss << "WARNING: " << messages_by_level_[2] << "\n";
  oss << "ERROR: " << messages_by_level_[3] << "\n";
  oss << "FATAL: " << messages_by_level_[4] << "\n";

  if (duration.count() > 0) {
    oss << "平均消息/秒: " << (total_messages_ / duration.count()) << "\n";
  }

  return oss.str();
}

void Logger::ProcessLogEntry(const LogEntry& entry) {
  // 检查是否应该输出
  if (config_.target == LogTarget::kConsoleOnly || config_.target == LogTarget::kBoth) {
    if (ShouldLog(entry.level, LogTarget::kConsoleOnly)) {
      OutputToConsole(entry);
    }
  }

  if (config_.target == LogTarget::kFileOnly || config_.target == LogTarget::kBoth) {
    if (ShouldLog(entry.level, LogTarget::kFileOnly)) {
      OutputToFile(entry);
    }
  }

  // 调用自定义处理器
  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    for (auto& handler : handlers_) {
      try {
        handler(entry);
      } catch (const std::exception& e) {
        std::cerr << "Handler exception: " << e.what() << std::endl;
      }
    }
  }
}

void Logger::OutputToConsole(const LogEntry& entry) {
  std::string formatted = FormatMessage(entry, config_.enable_colors);

  if (entry.level >= LogLevel::kError) {
    std::cerr << formatted << std::endl;
  } else {
    std::cout << formatted << std::endl;
  }
}

void Logger::OutputToFile(const LogEntry& entry) {
  std::lock_guard<std::mutex> lock(file_mutex_);

  if (!log_file_.is_open()) {
    return;
  }

  // 检查文件大小并轮转
  RotateLogFile();

  std::string formatted = FormatMessage(entry, false);
  log_file_ << formatted << std::endl;
  log_file_.flush();
}

std::string Logger::FormatMessage(const LogEntry& entry, bool use_colors) const {
  std::ostringstream oss;

  // 时间戳
  if (config_.enable_timestamps) {
    auto time_t = std::chrono::system_clock::to_time_t(entry.timestamp);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(entry.timestamp.time_since_epoch()) % 1000;

    oss << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
  }

  // 日志级别
  std::string level_str = GetLevelString(entry.level);
  if (use_colors) {
    oss << GetLevelColor(entry.level) << "[" << level_str << "]" << kColorReset << " ";
  } else {
    oss << "[" << level_str << "] ";
  }

  // 线程ID（如果需要）
  std::ostringstream thread_ss;
  thread_ss << entry.thread_id;
  oss << "[" << thread_ss.str().substr(0, 8) << "] ";

  // 消息内容
  oss << entry.message;

  // 源文件信息（DEBUG级别）
  if (entry.level == LogLevel::kDebug && !entry.file.empty()) {
    std::string filename = entry.file;
    size_t pos = filename.find_last_of("/\\");
    if (pos != std::string::npos) {
      filename = filename.substr(pos + 1);
    }
    oss << " (" << filename << ":" << entry.line;
    if (!entry.function.empty()) {
      oss << " in " << entry.function << "()";
    }
    oss << ")";
  }

  return oss.str();
}

std::string Logger::GetLevelString(LogLevel level) const {
  switch (level) {
    case LogLevel::kDebug:
      return "DEBUG";
    case LogLevel::kInfo:
      return "INFO ";
    case LogLevel::kWarning:
      return "WARN ";
    case LogLevel::kError:
      return "ERROR";
    case LogLevel::kFatal:
      return "FATAL";
    default:
      return "UNKNO";
  }
}

std::string Logger::GetLevelColor(LogLevel level) const {
  switch (level) {
    case LogLevel::kDebug:
      return kColorDebug;
    case LogLevel::kInfo:
      return kColorInfo;
    case LogLevel::kWarning:
      return kColorWarning;
    case LogLevel::kError:
      return kColorError;
    case LogLevel::kFatal:
      return kColorFatal;
    default:
      return kColorReset;
  }
}

bool Logger::CreateLogDirectory() {
  try {
    std::filesystem::create_directories(config_.log_directory);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to create log directory: " << e.what() << std::endl;
    return false;
  }
}

std::string Logger::GenerateLogFilename() const {
  if (!config_.log_filename.empty()) {
    return config_.log_directory + "/" + config_.log_filename;
  }

  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);

  std::ostringstream oss;
  oss << config_.log_directory << "/lidar_camera_calib_";
  oss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
  oss << ".log";

  return oss.str();
}

void Logger::RotateLogFile() {
  if (!log_file_.is_open()) {
    return;
  }

  // 检查文件大小
  auto current_pos = log_file_.tellp();
  if (current_pos < 0 || static_cast<size_t>(current_pos) < config_.max_file_size) {
    return;
  }

  // 关闭当前文件
  log_file_.close();

  // 生成新文件名
  current_log_file_ = GenerateLogFilename();

  // 打开新文件
  log_file_.open(current_log_file_, std::ios::app);
  if (!log_file_.is_open()) {
    std::cerr << "Failed to open rotated log file: " << current_log_file_ << std::endl;
    return;
  }

  // 清理旧文件
  CleanupOldFiles();
}

void Logger::CleanupOldFiles() {
  try {
    std::vector<std::filesystem::path> log_files;

    for (const auto& entry : std::filesystem::directory_iterator(config_.log_directory)) {
      if (entry.is_regular_file()) {
        std::string filename = entry.path().filename().string();
        if (filename.find("lidar_camera_calib_") == 0 && filename.find(".log") != std::string::npos) {
          log_files.push_back(entry.path());
        }
      }
    }

    // 按修改时间排序
    std::sort(log_files.begin(), log_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
      return std::filesystem::last_write_time(a) > std::filesystem::last_write_time(b);
    });

    // 删除多余文件
    while (log_files.size() > static_cast<size_t>(config_.max_files)) {
      std::filesystem::remove(log_files.back());
      log_files.pop_back();
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to cleanup old log files: " << e.what() << std::endl;
  }
}

void Logger::AsyncWorker() {
  while (!shutdown_requested_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    queue_condition_.wait(lock, [this] { return !log_queue_.empty() || shutdown_requested_; });

    while (!log_queue_.empty()) {
      LogEntry entry = log_queue_.front();
      log_queue_.pop();
      lock.unlock();

      ProcessLogEntry(entry);

      lock.lock();
    }
  }

  // 处理剩余的日志条目
  std::lock_guard<std::mutex> lock(queue_mutex_);
  while (!log_queue_.empty()) {
    LogEntry entry = log_queue_.front();
    log_queue_.pop();
    ProcessLogEntry(entry);
  }
}

bool Logger::ShouldLog(LogLevel level, LogTarget target) const {
  LogLevel threshold;

  if (target == LogTarget::kConsoleOnly) {
    threshold = config_.console_level;
  } else {
    threshold = config_.file_level;
  }

  return level >= threshold;
}