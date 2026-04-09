#include "exception_handler.h"

#include <iostream>
#include <sstream>
#include <typeinfo>

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wcast-align"
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/**
 * @file exception_handler.cpp
 * @brief 统一异常处理系统实现
 */

namespace lidar_camera_calib {

// 全局异常处理器实例
ExceptionHandler g_exception_handler;

ExceptionHandler::ExceptionHandler() : verbose_(false), quiet_(false) {}

ExceptionHandler::~ExceptionHandler() = default;

ExceptionHandleResult ExceptionHandler::HandleException(const std::exception& e) const {
  ExceptionHandleResult result;

  // 按异常类型进行特定处理
  try {
    // 尝试转换为数据集异常
    const DatasetValidationError& dataset_error = dynamic_cast<const DatasetValidationError&>(e);
    return HandleDatasetException(dataset_error);
  } catch (const std::bad_cast&) {
    // 不是数据集异常，继续尝试其他类型
  }

  try {
    // 尝试转换为PCD合并异常
    const PcdMergeError& pcd_error = dynamic_cast<const PcdMergeError&>(e);
    return HandlePcdMergeException(pcd_error);
  } catch (const std::bad_cast&) {
    // 不是PCD合并异常，继续尝试其他类型
  }

  try {
    // 尝试转换为视频提取异常
    const VideoExtractionError& video_error = dynamic_cast<const VideoExtractionError&>(e);
    return HandleVideoExtractionException(video_error);
  } catch (const std::bad_cast&) {
    // 不是视频提取异常，使用通用处理
  }

  // 通用异常处理
  result.handled = true;
  result.exit_code = 1;
  result.error_message = e.what();
  result.user_message = "An unexpected error occurred: " + std::string(e.what());

  return result;
}

void ExceptionHandler::SetVerbose(bool verbose) { verbose_ = verbose; }

void ExceptionHandler::SetQuiet(bool quiet) { quiet_ = quiet; }

void ExceptionHandler::SetErrorCallback(std::function<void(const std::string&)> callback) {
  error_callback_ = callback;
}

void ExceptionHandler::ReportError(const std::string& message, bool is_fatal) const {
  if (!quiet_) {
    LogMessage(is_fatal ? "FATAL" : "ERROR", message);
  }

  if (error_callback_) {
    error_callback_(message);
  }
}

void ExceptionHandler::ReportWarning(const std::string& message) const {
  if (!quiet_) {
    LogMessage("WARNING", message);
  }
}

std::string ExceptionHandler::GetUserFriendlyMessage(const std::exception& e) const {
  ExceptionHandleResult result = HandleException(e);
  return result.user_message;
}

ExceptionHandleResult ExceptionHandler::HandleDatasetException(const DatasetValidationError& e) const {
  ExceptionHandleResult result;
  result.handled = true;
  result.error_message = e.what();

  // 根据具体的数据集异常类型提供不同的处理
  try {
    const CameraDirectoryNotFound& camera_error = dynamic_cast<const CameraDirectoryNotFound&>(e);
    result.exit_code = 1;
    result.user_message = "Camera directory not found. Please check your dataset structure and camera ID filter.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const LidarDirectoryNotFound& lidar_error = dynamic_cast<const LidarDirectoryNotFound&>(e);
    result.exit_code = 1;
    result.user_message =
        "Lidar directory not found. Please ensure your dataset has a 'lidar/' directory with PCD files.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const InsufficientPcdFiles& pcd_error = dynamic_cast<const InsufficientPcdFiles&>(e);
    result.exit_code = 1;
    result.user_message =
        "Insufficient PCD files for processing. Please check your lidar directory contains valid PCD files.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const InvalidDatasetStructure& structure_error = dynamic_cast<const InvalidDatasetStructure&>(e);
    result.exit_code = 1;
    result.user_message =
        "Invalid dataset structure. Expected 'cameras/' and 'lidar/' directories with proper file organization.";
    return result;
  } catch (const std::bad_cast&) {
  }

  // 通用数据集错误
  result.exit_code = 1;
  result.user_message = "Dataset validation failed. Please check your dataset structure and file paths.";
  return result;
}

ExceptionHandleResult ExceptionHandler::HandlePcdMergeException(const PcdMergeError& e) const {
  ExceptionHandleResult result;
  result.handled = true;
  result.error_message = e.what();

  // 根据具体的PCD合并异常类型提供不同的处理
  try {
    const PcdFileCorrupted& corrupt_error = dynamic_cast<const PcdFileCorrupted&>(e);
    result.exit_code = 2;
    result.user_message = "PCD file is corrupted or unreadable. Please check the file integrity and format.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const PcdFormatIncompatible& format_error = dynamic_cast<const PcdFormatIncompatible&>(e);
    result.exit_code = 2;
    result.user_message = "PCD file format is incompatible. Please ensure all PCD files use supported point types.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const MemoryLimitExceeded& memory_error = dynamic_cast<const MemoryLimitExceeded&>(e);
    result.exit_code = 3;
    result.user_message =
        "Memory limit exceeded during PCD merging. Try reducing --max-pcds or increasing system memory.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const InsufficientDiskSpace& disk_error = dynamic_cast<const InsufficientDiskSpace&>(e);
    result.exit_code = 3;
    result.user_message = "Insufficient disk space for PCD processing. Please free up disk space and try again.";
    return result;
  } catch (const std::bad_cast&) {
  }

  // 通用PCD合并错误
  result.exit_code = 2;
  result.user_message = "PCD file merging failed. Please check file formats and system resources.";
  return result;
}

ExceptionHandleResult ExceptionHandler::HandleVideoExtractionException(const VideoExtractionError& e) const {
  ExceptionHandleResult result;
  result.handled = true;
  result.error_message = e.what();

  // 根据具体的视频提取异常类型提供不同的处理
  try {
    const VideoFileNotFound& not_found_error = dynamic_cast<const VideoFileNotFound&>(e);
    result.exit_code = 1;
    result.user_message = "Video file not found. Please check the file path and permissions.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const VideoFormatUnsupported& format_error = dynamic_cast<const VideoFormatUnsupported&>(e);
    result.exit_code = 2;
    result.user_message = "Video format not supported. Please use MP4, AVI, MOV, or MKV format.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const VideoCorrupted& corrupt_error = dynamic_cast<const VideoCorrupted&>(e);
    result.exit_code = 2;
    result.user_message = "Video file is corrupted or unreadable. Please check file integrity.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const FrameIndexOutOfRange& frame_error = dynamic_cast<const FrameIndexOutOfRange&>(e);
    result.exit_code = 4;
    result.user_message = "Frame index out of range. Please specify a valid frame index within the video length.";
    return result;
  } catch (const std::bad_cast&) {
  }

  try {
    const OutputPathInvalid& path_error = dynamic_cast<const OutputPathInvalid&>(e);
    result.exit_code = 4;
    result.user_message = "Invalid output path. Please check directory permissions and path validity.";
    return result;
  } catch (const std::bad_cast&) {
  }

  // 通用视频提取错误
  result.exit_code = 2;
  result.user_message = "Video frame extraction failed. Please check video file and extraction settings.";
  return result;
}

int ExceptionHandler::GetSuggestedExitCode(const std::exception& e) const {
  ExceptionHandleResult result = HandleException(e);
  return result.exit_code;
}

std::string ExceptionHandler::GetExceptionTypeName(const std::exception& e) const { return typeid(e).name(); }

void ExceptionHandler::LogMessage(const std::string& level, const std::string& message) const {
  if (quiet_) {
    return;
  }

  std::string prefix;
  if (level == "ERROR" || level == "FATAL") {
    prefix = "\033[31m[" + level + "]\033[0m ";  // 红色
  } else if (level == "WARNING") {
    prefix = "\033[33m[" + level + "]\033[0m ";  // 黄色
  } else if (level == "DETAIL") {
    prefix = "\033[36m[" + level + "]\033[0m ";  // 青色
  } else {
    prefix = "[" + level + "] ";
  }

  std::cerr << prefix << message << std::endl;
}

}  // namespace lidar_camera_calib