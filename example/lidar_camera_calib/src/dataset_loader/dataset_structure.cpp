#include "dataset_structure.h"
#include <filesystem>
#include <algorithm>
#include <sstream>

namespace lidar_camera_calib {

// CameraDirectory 实现

std::string CameraDirectory::GetPrimaryVideo() const { return mp4_files.empty() ? std::string() : mp4_files.front(); }

bool CameraDirectory::HasVideos() const { return !mp4_files.empty(); }

bool CameraDirectory::Validate() const {
  // 检查相机ID格式
  if (camera_id.empty() || camera_id.find("camera_") != 0) {
    return false;
  }

  // 检查目录是否存在
  if (!std::filesystem::exists(directory_path) || !std::filesystem::is_directory(directory_path)) {
    return false;
  }

  // 检查是否至少有一个MP4文件
  if (mp4_files.empty()) {
    return false;
  }

  // 验证所有MP4文件是否存在
  for (const auto& mp4_file : mp4_files) {
    if (!std::filesystem::exists(mp4_file)) {
      return false;
    }
  }

  return true;
}

// LidarDirectory 实现

std::vector<std::string> LidarDirectory::GetFilesByLimit(int max_count) const {
  if (max_count < 0) {
    return pcd_files;
  }

  std::vector<std::string> limited_files;
  int count = std::min(max_count, static_cast<int>(pcd_files.size()));
  limited_files.reserve(count);

  for (int i = 0; i < count; ++i) {
    limited_files.push_back(pcd_files[i]);
  }

  return limited_files;
}

size_t LidarDirectory::GetTotalFileCount() const { return pcd_files.size(); }

bool LidarDirectory::HasFiles() const { return !pcd_files.empty(); }

bool LidarDirectory::Validate() const {
  // 检查目录是否存在
  if (!std::filesystem::exists(directory_path) || !std::filesystem::is_directory(directory_path)) {
    return false;
  }

  // 检查是否有PCD文件
  if (pcd_files.empty()) {
    return false;
  }

  // 验证所有PCD文件是否存在且有正确扩展名
  for (const auto& pcd_file : pcd_files) {
    if (!std::filesystem::exists(pcd_file)) {
      return false;
    }

    std::string extension = std::filesystem::path(pcd_file).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    if (extension != ".pcd") {
      return false;
    }
  }

  return true;
}

// DatasetStructure 实现

bool DatasetStructure::Validate() const {
  // 检查根目录是否存在
  if (!std::filesystem::exists(root_path) || !std::filesystem::is_directory(root_path)) {
    return false;
  }

  // 检查是否有相机目录
  if (cameras.empty()) {
    return false;
  }

  // 验证所有相机目录
  for (const auto& camera : cameras) {
    if (!camera.Validate()) {
      return false;
    }
  }

  // 验证激光雷达目录
  if (!lidar.Validate()) {
    return false;
  }

  return true;
}

std::vector<std::string> DatasetStructure::GetValidationErrors() const {
  std::vector<std::string> errors;

  // 检查根目录
  if (!std::filesystem::exists(root_path)) {
    errors.push_back("Root directory does not exist: " + root_path);
  } else if (!std::filesystem::is_directory(root_path)) {
    errors.push_back("Root path is not a directory: " + root_path);
  }

  // 检查相机目录
  if (cameras.empty()) {
    errors.push_back("No camera directories found");
  } else {
    for (size_t i = 0; i < cameras.size(); ++i) {
      const auto& camera = cameras[i];
      if (!camera.Validate()) {
        if (camera.camera_id.empty() || camera.camera_id.find("camera_") != 0) {
          errors.push_back("Invalid camera ID format: " + camera.camera_id);
        }
        if (!std::filesystem::exists(camera.directory_path)) {
          errors.push_back("Camera directory does not exist: " + camera.directory_path);
        }
        if (camera.mp4_files.empty()) {
          errors.push_back("No MP4 files found in camera directory: " + camera.directory_path);
        }
      }
    }
  }

  // 检查激光雷达目录
  if (!lidar.Validate()) {
    if (!std::filesystem::exists(lidar.directory_path)) {
      errors.push_back("Lidar directory does not exist: " + lidar.directory_path);
    }
    if (lidar.pcd_files.empty()) {
      errors.push_back("No PCD files found in lidar directory: " + lidar.directory_path);
    }
  }

  return errors;
}

const CameraDirectory* DatasetStructure::FindCamera(const std::string& camera_id) const {
  auto it = std::find_if(cameras.begin(), cameras.end(),
                         [&camera_id](const CameraDirectory& camera) { return camera.camera_id == camera_id; });

  return (it != cameras.end()) ? &(*it) : nullptr;
}

std::string DatasetStructure::GetSummary() const {
  std::ostringstream oss;
  oss << "Dataset Summary:\n";
  oss << "  Root: " << root_path << "\n";
  oss << "  Cameras: " << cameras.size() << "\n";

  for (const auto& camera : cameras) {
    oss << "    " << camera.camera_id << ": " << camera.mp4_files.size() << " MP4 files\n";
  }

  oss << "  Lidar: " << lidar.pcd_files.size() << " PCD files\n";

  return oss.str();
}

std::vector<TimestampMatch> DatasetStructure::GetTimestampMatches(const std::string& camera_id,
                                                                 uint64_t max_time_diff_ms) const {
  std::vector<TimestampMatch> matches;
  uint64_t max_time_diff_ns = max_time_diff_ms * 1000000ULL; // 转换为纳秒

  // 处理指定相机或所有相机
  std::vector<const CameraDirectory*> target_cameras;
  if (camera_id.empty()) {
    for (const auto& camera : cameras) {
      target_cameras.push_back(&camera);
    }
  } else {
    const CameraDirectory* camera = FindCamera(camera_id);
    if (camera) {
      target_cameras.push_back(camera);
    }
  }

  // 为每个相机寻找匹配的激光雷达文件
  for (const auto* camera : target_cameras) {
    for (const auto& camera_file : camera->timestamp_files) {
      TimestampMatch best_match;
      uint64_t best_time_diff = UINT64_MAX;

      // 在激光雷达文件中寻找最佳匹配
      for (const auto& lidar_file : lidar.timestamp_files) {
        uint64_t time_diff = (camera_file.timestamp_ns > lidar_file.timestamp_ns) ?
                            camera_file.timestamp_ns - lidar_file.timestamp_ns :
                            lidar_file.timestamp_ns - camera_file.timestamp_ns;

        if (time_diff <= max_time_diff_ns && time_diff < best_time_diff) {
          best_time_diff = time_diff;
          best_match = TimestampMatch(camera_file, lidar_file);
        }
      }

      if (best_match.is_valid) {
        matches.push_back(best_match);
      }
    }
  }

  // 按时间戳排序
  std::sort(matches.begin(), matches.end(),
           [](const TimestampMatch& a, const TimestampMatch& b) {
             return a.camera_file.timestamp_ns < b.camera_file.timestamp_ns;
           });

  return matches;
}

}  // namespace lidar_camera_calib