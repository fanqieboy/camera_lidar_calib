#include "pcd_merger.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <fstream>

/**
 * @file pcd_merger.cpp
 * @brief PCD文件合并器类实现
 */

namespace lidar_camera_calib {

// MemoryMonitor implementation
MemoryMonitor::MemoryMonitor() : limit_bytes_(0), has_limit_(false) {}

void MemoryMonitor::SetLimit(size_t limit_bytes) {
  limit_bytes_ = limit_bytes;
  has_limit_ = true;
}

void MemoryMonitor::CheckUsage() const {
  if (!has_limit_) {
    return;
  }

  size_t current = GetCurrentUsage();
  if (current > limit_bytes_) {
    throw MemoryLimitExceeded(current, limit_bytes_);
  }
}

size_t MemoryMonitor::GetCurrentUsage() const {
  // Simple estimation based on virtual memory
  // In a real implementation, you might want to use more sophisticated memory tracking
  try {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
      if (line.find("VmRSS:") == 0) {
        std::istringstream iss(line);
        std::string key, value, unit;
        iss >> key >> value >> unit;
        return std::stoull(value) * 1024;  // Convert KB to bytes
      }
    }
  } catch (...) {
    // Fallback: return 0 if unable to read memory usage
  }
  return 0;
}

size_t MemoryMonitor::GetLimit() const { return limit_bytes_; }

bool MemoryMonitor::HasLimit() const { return has_limit_; }

// Exception implementations
PcdMergeError::PcdMergeError(const std::string& message) : message_(message) {}

const char* PcdMergeError::what() const noexcept { return message_.c_str(); }

PcdFileCorrupted::PcdFileCorrupted(const std::string& filename) : PcdMergeError("PCD file corrupted: " + filename) {}

PcdFormatIncompatible::PcdFormatIncompatible(const std::string& filename, const std::string& details)
    : PcdMergeError("PCD format incompatible (" + filename + "): " + details) {}

MemoryLimitExceeded::MemoryLimitExceeded(size_t current_usage, size_t limit)
    : PcdMergeError("Memory limit exceeded: using " + std::to_string(current_usage) + " bytes, limit is " +
                    std::to_string(limit) + " bytes") {}

InsufficientDiskSpace::InsufficientDiskSpace(const std::string& details)
    : PcdMergeError("Insufficient disk space: " + details) {}

// PcdMerger implementation
PcdMerger::PcdMerger()
    : memory_limit_mb_(0),
      max_files_(-1),
      downsample_voxel_size_(0.0f),
      memory_monitor_(std::make_unique<MemoryMonitor>()) {}

PcdMerger::~PcdMerger() = default;

void PcdMerger::SetMemoryLimit(size_t limit_mb) {
  memory_limit_mb_ = limit_mb;
  memory_monitor_->SetLimit(limit_mb * 1024 * 1024);  // Convert MB to bytes
}

void PcdMerger::SetMaxFiles(int max_count) { max_files_ = max_count; }

void PcdMerger::SetDownsampleVoxelSize(float size) { downsample_voxel_size_ = size; }

MergedPointCloud PcdMerger::MergeFiles(const std::vector<std::string>& pcd_files) {
  if (pcd_files.empty()) {
    throw PcdMergeError("No PCD files provided for merging");
  }

  // Check memory estimation first
  size_t estimated_memory = EstimateMemoryUsage(pcd_files);
  if (memory_monitor_->HasLimit() && estimated_memory > memory_monitor_->GetLimit()) {
    throw MemoryLimitExceeded(estimated_memory, memory_monitor_->GetLimit());
  }

  MergedPointCloud result;
  result.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // Apply file limit if set
  std::vector<std::string> files_to_process = pcd_files;
  if (max_files_ > 0 && static_cast<int>(files_to_process.size()) > max_files_) {
    files_to_process.resize(max_files_);
  }

  // Process files one by one using streaming strategy
  for (const std::string& filename : files_to_process) {
    try {
      // Validate file before processing
      if (!ValidatePcdFile(filename)) {
        throw PcdFileCorrupted(filename);
      }

      // Check memory usage before loading next file
      memory_monitor_->CheckUsage();

      // Load PCD file
      auto single_cloud = LoadPcdFile(filename);
      size_t point_count = single_cloud->size();

      // Merge with result cloud
      MergePointCloud(result.cloud, single_cloud);

      // Add statistics
      result.source_files.push_back(filename);
      result.file_point_counts.push_back(point_count);

      // Clear single_cloud to free memory immediately
      single_cloud.reset();

    } catch (const PcdMergeError&) {
      throw;  // Re-throw merge errors as-is
    } catch (const std::exception& e) {
      throw PcdMergeError("Failed to process file " + filename + ": " + e.what());
    }
  }

  // Apply downsampling if configured
  if (downsample_voxel_size_ > 0.0f) {
    result.cloud = ApplyDownsampling(result.cloud);
  }

  return result;
}

MergedPointCloud PcdMerger::MergeDirectory(const std::string& directory_path, int max_files) {
  if (!std::filesystem::exists(directory_path) || !std::filesystem::is_directory(directory_path)) {
    throw PcdMergeError("Directory does not exist: " + directory_path);
  }

  // Get PCD files in alphabetical order
  std::vector<std::string> pcd_files = GetPcdFilesInDirectory(directory_path);

  if (pcd_files.empty()) {
    throw PcdMergeError("No PCD files found in directory: " + directory_path);
  }

  // Apply file limit
  if (max_files > 0 && static_cast<int>(pcd_files.size()) > max_files) {
    pcd_files.resize(max_files);
  }

  // Delegate to MergeFiles
  return MergeFiles(pcd_files);
}

size_t PcdMerger::EstimateMemoryUsage(const std::vector<std::string>& pcd_files) const {
  size_t total_estimate = 0;

  for (const std::string& filename : pcd_files) {
    total_estimate += EstimateSingleFileMemory(filename);
  }

  // Add overhead for merged cloud (assume 20% overhead)
  total_estimate = static_cast<size_t>(total_estimate * 1.2);

  return total_estimate;
}

std::string PcdMerger::GetConfigSummary() const {
  std::ostringstream oss;
  oss << "PcdMerger Config:\n";
  oss << "  Memory Limit: ";
  if (memory_limit_mb_ > 0) {
    oss << memory_limit_mb_ << " MB";
  } else {
    oss << "No limit";
  }
  oss << "\n  Max Files: ";
  if (max_files_ > 0) {
    oss << max_files_;
  } else {
    oss << "No limit";
  }
  oss << "\n  Downsample Voxel Size: ";
  if (downsample_voxel_size_ > 0.0f) {
    oss << downsample_voxel_size_ << " m";
  } else {
    oss << "Disabled";
  }
  return oss.str();
}

void PcdMerger::Reset() {
  memory_limit_mb_ = 0;
  max_files_ = -1;
  downsample_voxel_size_ = 0.0f;
  memory_monitor_ = std::make_unique<MemoryMonitor>();
}

bool PcdMerger::ValidatePcdFile(const std::string& filename) const {
  if (!std::filesystem::exists(filename)) {
    throw PcdFileCorrupted(filename);
  }

  // Try to open file to check basic readability
  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }

  // Read first few bytes to check for PCD signature
  std::string header;
  std::getline(file, header);
  if (header.find("# .PCD") != 0) {
    throw PcdFormatIncompatible(filename, "Invalid PCD header");
  }

  return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PcdMerger::LoadPcdFile(const std::string& filename) const {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1) {
    throw PcdFileCorrupted(filename);
  }

  if (cloud->empty()) {
    throw PcdFormatIncompatible(filename, "Empty point cloud");
  }

  return cloud;
}

void PcdMerger::MergePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr target,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr source) const {
  if (!source || source->empty()) {
    return;
  }

  // Simple concatenation merge
  *target += *source;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PcdMerger::ApplyDownsampling(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) const {
  if (downsample_voxel_size_ <= 0.0f || !cloud || cloud->empty()) {
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(downsample_voxel_size_, downsample_voxel_size_, downsample_voxel_size_);
  voxel_filter.filter(*filtered_cloud);

  return filtered_cloud;
}

std::vector<std::string> PcdMerger::GetPcdFilesInDirectory(const std::string& directory_path) const {
  std::vector<std::string> pcd_files;

  try {
    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
      if (entry.is_regular_file()) {
        std::string filename = entry.path().filename().string();
        std::string extension = entry.path().extension().string();

        // Convert extension to lowercase
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        if (extension == ".pcd") {
          pcd_files.push_back(entry.path().string());
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    throw PcdMergeError("Failed to scan directory " + directory_path + ": " + e.what());
  }

  // Sort alphabetically
  std::sort(pcd_files.begin(), pcd_files.end());

  return pcd_files;
}

size_t PcdMerger::EstimateSingleFileMemory(const std::string& filename) const {
  try {
    size_t file_size = std::filesystem::file_size(filename);
    // Rough estimation: PCD files are typically compressed
    // Assume the in-memory point cloud will be 3-4 times larger
    return file_size * 4;
  } catch (const std::filesystem::filesystem_error&) {
    // Fallback estimation: assume 10MB per file
    return 10 * 1024 * 1024;
  }
}

bool PcdMerger::CheckDiskSpace(size_t required_space) const {
  try {
    auto space_info = std::filesystem::space(".");
    return space_info.available >= required_space;
  } catch (const std::filesystem::filesystem_error&) {
    // Assume space is available if we can't check
    return true;
  }
}

}  // namespace lidar_camera_calib