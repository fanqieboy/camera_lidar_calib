#include "merged_point_cloud.h"
#include <sstream>
#include <numeric>

namespace lidar_camera_calib {

// MergedPointCloud 实现

MergedPointCloud::MergedPointCloud() : cloud(new pcl::PointCloud<pcl::PointXYZI>()) {}

MergedPointCloud::MergedPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr) : cloud(cloud_ptr) {
  if (!cloud) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
}

size_t MergedPointCloud::GetTotalPoints() const { return cloud ? cloud->size() : 0; }

std::string MergedPointCloud::GetMergeSummary() const {
  std::ostringstream oss;
  oss << "Merged Point Cloud Summary:\n";
  oss << "  Total Points: " << GetTotalPoints() << "\n";
  oss << "  Source Files: " << source_files.size() << "\n";

  for (size_t i = 0; i < source_files.size() && i < file_point_counts.size(); ++i) {
    oss << "    " << source_files[i] << ": " << file_point_counts[i] << " points\n";
  }

  return oss.str();
}

bool MergedPointCloud::Validate() const {
  // 检查点云指针
  if (!cloud) {
    return false;
  }

  // 检查统计数据一致性
  if (source_files.size() != file_point_counts.size()) {
    return false;
  }

  // 检查总点数是否与统计数据一致
  size_t calculated_total = std::accumulate(file_point_counts.begin(), file_point_counts.end(), 0UL);
  if (calculated_total != GetTotalPoints()) {
    return false;
  }

  return true;
}

size_t MergedPointCloud::GetFilePointCount(size_t file_index) const {
  if (file_index >= file_point_counts.size()) {
    return 0;
  }
  return file_point_counts[file_index];
}

size_t MergedPointCloud::GetSourceFileCount() const { return source_files.size(); }

bool MergedPointCloud::IsEmpty() const { return !cloud || cloud->empty(); }

void MergedPointCloud::Clear() {
  if (cloud) {
    cloud->clear();
  }
  source_files.clear();
  file_point_counts.clear();
}

void MergedPointCloud::AddFileStatistics(const std::string& file_path, size_t point_count) {
  source_files.push_back(file_path);
  file_point_counts.push_back(point_count);
}

}  // namespace lidar_camera_calib