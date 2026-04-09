#include "dataset_loader.h"
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <regex>
#include <chrono>
#include <iomanip>

/**
 * @file dataset_loader.cpp
 * @brief 数据集加载器类实现
 */

namespace lidar_camera_calib {

// ValidationResult implementation
ValidationResult::ValidationResult() : success(false) {}

void ValidationResult::AddError(const std::string& error) {
  errors.push_back(error);
  success = false;
}

void ValidationResult::AddWarning(const std::string& warning) { warnings.push_back(warning); }

bool ValidationResult::HasErrors() const { return !errors.empty(); }

std::string ValidationResult::GetSummary() const {
  std::ostringstream oss;
  oss << "Validation ";
  if (success) {
    oss << "SUCCESS";
  } else {
    oss << "FAILED";
  }
  oss << " - Errors: " << errors.size() << ", Warnings: " << warnings.size();
  return oss.str();
}

// Exception implementations
DatasetValidationError::DatasetValidationError(const std::string& message) : message_(message) {}

const char* DatasetValidationError::what() const noexcept { return message_.c_str(); }

CameraDirectoryNotFound::CameraDirectoryNotFound(const std::string& camera_id)
    : DatasetValidationError("Camera directory not found: " + camera_id) {}

LidarDirectoryNotFound::LidarDirectoryNotFound(const std::string& directory_path)
    : DatasetValidationError("Lidar directory not found: " + directory_path) {}

InsufficientPcdFiles::InsufficientPcdFiles(int found_count, int required_count)
    : DatasetValidationError("Insufficient PCD files: found " + std::to_string(found_count) + ", required at least " +
                             std::to_string(required_count)) {}

InvalidDatasetStructure::InvalidDatasetStructure(const std::string& details)
    : DatasetValidationError("Invalid dataset structure: " + details) {}

// DatasetLoader implementation
DatasetLoader::DatasetLoader(const std::string& root_path) : root_path_(root_path), max_pcd_files_(-1), max_time_diff_ms_(100) {
  if (root_path_.empty()) {
    throw std::invalid_argument("Root path cannot be empty");
  }

  if (!DirectoryExists(root_path_)) {
    throw std::invalid_argument("Root directory does not exist: " + root_path_);
  }
}

void DatasetLoader::SetCameraId(const std::string& camera_id) { camera_id_ = camera_id; }

void DatasetLoader::SetMaxPcdFiles(int max_count) { max_pcd_files_ = max_count; }

void DatasetLoader::SetMaxTimeDifference(uint64_t max_time_diff_ms) { max_time_diff_ms_ = max_time_diff_ms; }

ValidationResult DatasetLoader::Validate() const {
  ValidationResult result;

  // Check root directory
  if (!DirectoryExists(root_path_)) {
    result.AddError("Root directory does not exist: " + root_path_);
    return result;
  }

  // Check cameras directory
  std::string cameras_dir = root_path_ + "/cameras";
  if (!DirectoryExists(cameras_dir)) {
    result.AddError("Cameras directory not found: " + cameras_dir);
  } else {
    // Check for specific camera or any camera directories
    auto camera_dirs = ScanCameraDirectories();
    if (camera_dirs.empty()) {
      if (camera_id_.empty()) {
        result.AddError("No camera directories found in: " + cameras_dir);
      } else {
        result.AddError("Camera directory not found: " + camera_id_);
      }
    } else {
      // Check each camera directory validity
      for (const auto& cam_dir : camera_dirs) {
        if (!cam_dir.Validate()) {
          result.AddWarning("Camera " + cam_dir.camera_id + " has no valid video files");
        }
      }
    }
  }

  // Check lidar directory
  std::string lidar_dir = root_path_ + "/lidar";
  if (!DirectoryExists(lidar_dir)) {
    result.AddError("Lidar directory not found: " + lidar_dir);
  } else {
    auto lidar_structure = ScanLidarDirectory();
    if (!lidar_structure.HasFiles()) {
      result.AddError("No PCD files found in lidar directory");
    } else if (lidar_structure.GetTotalFileCount() == 0) {
      result.AddError("Lidar directory exists but contains no PCD files");
    } else {
      // Check PCD file count against limits
      int available_files = static_cast<int>(lidar_structure.GetTotalFileCount());
      if (max_pcd_files_ > 0 && available_files > max_pcd_files_) {
        result.AddWarning("Found " + std::to_string(available_files) + " PCD files, will use first " +
                          std::to_string(max_pcd_files_));
      }
    }
  }

  // If no errors, mark as successful
  if (!result.HasErrors()) {
    result.success = true;
  }

  return result;
}

DatasetStructure DatasetLoader::LoadDataset() const {
  // Validate first
  ValidationResult validation = Validate();
  if (!validation.success) {
    std::string error_msg = "Dataset validation failed:\n";
    for (const auto& error : validation.errors) {
      error_msg += "  ERROR: " + error + "\n";
    }
    throw DatasetValidationError(error_msg);
  }

  DatasetStructure dataset;
  dataset.root_path = root_path_;

  // Load camera directories
  dataset.cameras = ScanCameraDirectories();

  // Load lidar directory
  dataset.lidar = ScanLidarDirectory();

  return dataset;
}

std::string DatasetLoader::GetConfigSummary() const {
  std::ostringstream oss;
  oss << "DatasetLoader Config:\n";
  oss << "  Root Path: " << root_path_ << "\n";
  oss << "  Camera Filter: " << (camera_id_.empty() ? "All cameras" : camera_id_) << "\n";
  oss << "  Max PCD Files: " << (max_pcd_files_ == -1 ? "No limit" : std::to_string(max_pcd_files_));
  return oss.str();
}

std::vector<CameraDirectory> DatasetLoader::ScanCameraDirectories() const {
  std::vector<CameraDirectory> camera_dirs;
  std::string cameras_root = root_path_ + "/cameras";

  if (!DirectoryExists(cameras_root)) {
    return camera_dirs;
  }

  try {
    for (const auto& entry : std::filesystem::directory_iterator(cameras_root)) {
      if (entry.is_directory()) {
        std::string dir_name = entry.path().filename().string();

        // Check if this matches camera ID filter
        if (!camera_id_.empty() && dir_name != camera_id_) {
          continue;
        }

        // Check if this looks like a camera directory (camera_X pattern)
        if (dir_name.find("camera_") == 0) {
          CameraDirectory cam_dir;
          cam_dir.camera_id = dir_name;
          cam_dir.directory_path = entry.path().string();

          // Scan for MP4 files
          cam_dir.mp4_files = GetFilesInDirectory(cam_dir.directory_path, ".mp4");

          // Scan for TXT files (but don't process them)
          cam_dir.txt_files = GetFilesInDirectory(cam_dir.directory_path, ".txt");

          // 解析时间戳信息
          for (const auto& mp4_file : cam_dir.mp4_files) {
            std::string filename = std::filesystem::path(mp4_file).filename().string();
            TimestampInfo ts_info = ParseTimestamp(filename, mp4_file);
            if (ts_info.is_valid) {
              cam_dir.timestamp_files.push_back(ts_info);
            }
          }

          // 按时间戳排序
          std::sort(cam_dir.timestamp_files.begin(), cam_dir.timestamp_files.end(),
                   [](const TimestampInfo& a, const TimestampInfo& b) {
                     return a.timestamp_ns < b.timestamp_ns;
                   });

          camera_dirs.push_back(cam_dir);
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    throw DatasetValidationError("Failed to scan cameras directory: " + std::string(e.what()));
  }

  return camera_dirs;
}

LidarDirectory DatasetLoader::ScanLidarDirectory() const {
  LidarDirectory lidar_dir;
  lidar_dir.directory_path = root_path_ + "/lidar";

  if (!DirectoryExists(lidar_dir.directory_path)) {
    return lidar_dir;
  }

  // Scan for PCD files
  lidar_dir.pcd_files = GetFilesInDirectory(lidar_dir.directory_path, ".pcd");

  // 解析时间戳信息
  for (const auto& pcd_file : lidar_dir.pcd_files) {
    std::string filename = std::filesystem::path(pcd_file).filename().string();
    TimestampInfo ts_info = ParseTimestamp(filename, pcd_file);
    if (ts_info.is_valid) {
      lidar_dir.timestamp_files.push_back(ts_info);
    }
  }

  // 按时间戳排序
  std::sort(lidar_dir.timestamp_files.begin(), lidar_dir.timestamp_files.end(),
           [](const TimestampInfo& a, const TimestampInfo& b) {
             return a.timestamp_ns < b.timestamp_ns;
           });

  return lidar_dir;
}

bool DatasetLoader::DirectoryExists(const std::string& directory_path) const {
  try {
    return std::filesystem::exists(directory_path) && std::filesystem::is_directory(directory_path);
  } catch (const std::filesystem::filesystem_error&) {
    return false;
  }
}

std::vector<std::string> DatasetLoader::GetFilesInDirectory(const std::string& directory_path,
                                                            const std::string& extension) const {
  std::vector<std::string> files;

  try {
    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
      if (entry.is_regular_file()) {
        std::string filename = entry.path().filename().string();
        std::string file_ext = entry.path().extension().string();

        // Convert to lowercase for comparison
        std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower);

        if (file_ext == extension) {
          files.push_back(entry.path().string());
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    throw DatasetValidationError("Failed to scan directory " + directory_path + ": " + std::string(e.what()));
  }

  // Sort files alphabetically
  std::sort(files.begin(), files.end());

  return files;
}

bool DatasetLoader::IsTimestampFormat(const std::string& filename) {
  // 匹配格式: YYYYMMDDHHMMSS_NNNNNNNNN.ext
  // 例如: 20240618043634_981660770.mp4
  std::regex timestamp_pattern(R"(^\d{14}_\d{9}\.)");
  return std::regex_search(filename, timestamp_pattern);
}

TimestampInfo DatasetLoader::ParseTimestamp(const std::string& filename, const std::string& full_path) {
  if (!IsTimestampFormat(filename)) {
    return TimestampInfo(); // 返回无效的时间戳信息
  }

  try {
    // 提取日期部分和纳秒部分
    size_t underscore_pos = filename.find('_');
    size_t dot_pos = filename.find('.', underscore_pos);

    if (underscore_pos == std::string::npos || dot_pos == std::string::npos ||
        underscore_pos != 14 || (dot_pos - underscore_pos - 1) != 9) {
      return TimestampInfo();
    }

    std::string date_part = filename.substr(0, 14);     // YYYYMMDDHHMMSS
    std::string nano_part = filename.substr(15, 9);     // NNNNNNNNN

    uint64_t timestamp_ns = ConvertToNanoseconds(date_part, nano_part);
    if (timestamp_ns == 0) {
      return TimestampInfo();
    }

    return TimestampInfo(filename, full_path, timestamp_ns, date_part, nano_part);
  } catch (const std::exception&) {
    return TimestampInfo();
  }
}

uint64_t DatasetLoader::ConvertToNanoseconds(const std::string& date_part, const std::string& nano_part) {
  try {
    // 解析日期部分 YYYYMMDDHHMMSS
    if (date_part.length() != 14) return 0;

    int year = std::stoi(date_part.substr(0, 4));
    int month = std::stoi(date_part.substr(4, 2));
    int day = std::stoi(date_part.substr(6, 2));
    int hour = std::stoi(date_part.substr(8, 2));
    int minute = std::stoi(date_part.substr(10, 2));
    int second = std::stoi(date_part.substr(12, 2));

    // 验证日期时间有效性
    if (year < 1970 || year > 2100 || month < 1 || month > 12 ||
        day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) {
      return 0;
    }

    // 创建时间点
    std::tm tm_time = {};
    tm_time.tm_year = year - 1900;
    tm_time.tm_mon = month - 1;
    tm_time.tm_mday = day;
    tm_time.tm_hour = hour;
    tm_time.tm_min = minute;
    tm_time.tm_sec = second;

    // 转换为时间戳（秒）
    std::time_t time_seconds = std::mktime(&tm_time);
    if (time_seconds == -1) return 0;

    // 解析纳秒部分
    if (nano_part.length() != 9) return 0;
    uint64_t nanoseconds = std::stoull(nano_part);

    // 转换为纳秒级时间戳
    uint64_t timestamp_ns = static_cast<uint64_t>(time_seconds) * 1000000000ULL + nanoseconds;

    return timestamp_ns;
  } catch (const std::exception&) {
    return 0;
  }
}

TimestampInfo DatasetLoader::ParseTimestampPublic(const std::string& filename, const std::string& full_path) {
  return ParseTimestamp(filename, full_path);
}

}  // namespace lidar_camera_calib