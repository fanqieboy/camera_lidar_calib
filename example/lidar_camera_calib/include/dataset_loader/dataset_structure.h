#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <optional>

/**
 * @file dataset_structure.h
 * @brief 特定格式数据集目录结构的数据模型定义
 */

namespace lidar_camera_calib {

/**
 * @brief 时间戳信息结构
 */
struct TimestampInfo {
    std::string filename;           // 原始文件名
    std::string full_path;         // 完整文件路径
    uint64_t timestamp_ns;         // 纳秒级时间戳
    std::string date_part;         // 日期部分 (YYYYMMDDHHMMSS)
    std::string nano_part;         // 纳秒部分 (NNNNNNNNN)
    bool is_valid;                 // 时间戳是否有效

    /**
     * @brief 默认构造函数
     */
    TimestampInfo() : timestamp_ns(0), is_valid(false) {}

    /**
     * @brief 构造函数
     */
    TimestampInfo(const std::string& fname, const std::string& path,
                  uint64_t ts, const std::string& date, const std::string& nano)
        : filename(fname), full_path(path), timestamp_ns(ts),
          date_part(date), nano_part(nano), is_valid(true) {}
};

/**
 * @brief 时间戳匹配结果
 */
struct TimestampMatch {
    TimestampInfo camera_file;     // 匹配的相机文件
    TimestampInfo lidar_file;      // 匹配的激光雷达文件
    uint64_t time_diff_ns;         // 时间差（纳秒）
    bool is_valid;                 // 匹配是否有效

    /**
     * @brief 默认构造函数
     */
    TimestampMatch() : time_diff_ns(UINT64_MAX), is_valid(false) {}

    /**
     * @brief 构造函数
     */
    TimestampMatch(const TimestampInfo& camera, const TimestampInfo& lidar)
        : camera_file(camera), lidar_file(lidar), is_valid(true) {
        time_diff_ns = (camera.timestamp_ns > lidar.timestamp_ns) ?
                       camera.timestamp_ns - lidar.timestamp_ns :
                       lidar.timestamp_ns - camera.timestamp_ns;
    }
};

/**
 * @brief 单个相机目录结构
 *
 * 包含相机目录的基本信息和文件列表，支持MP4视频文件处理
 */
struct CameraDirectory {
    std::string camera_id;              // camera_0, camera_1, etc.
    std::string directory_path;         // 完整目录路径
    std::vector<std::string> mp4_files; // MP4视频文件列表
    std::vector<std::string> txt_files; // TXT标注文件（忽略处理）
    std::vector<TimestampInfo> timestamp_files; // 包含时间戳信息的文件列表

    /**
     * @brief 获取主要视频文件
     * @return 第一个MP4文件路径，若无则返回空字符串
     */
    std::string GetPrimaryVideo() const;

    /**
     * @brief 检查是否包含视频文件
     * @return true如果包含至少一个MP4文件
     */
    bool HasVideos() const;

    /**
     * @brief 验证相机目录结构
     * @return true如果目录结构有效
     */
    bool Validate() const;
};

/**
 * @brief 激光雷达目录结构
 *
 * 管理PCD文件列表，支持按文件名排序和数量限制
 */
struct LidarDirectory {
    std::string directory_path;         // lidar目录路径
    std::vector<std::string> pcd_files; // 按文件名排序的PCD文件列表
    std::vector<TimestampInfo> timestamp_files; // 包含时间戳信息的PCD文件列表

    /**
     * @brief 根据数量限制获取PCD文件列表
     * @param max_count 最大文件数量，-1表示不限制
     * @return 受限制的PCD文件路径列表
     */
    std::vector<std::string> GetFilesByLimit(int max_count = -1) const;

    /**
     * @brief 获取PCD文件总数
     * @return 文件总数
     */
    size_t GetTotalFileCount() const;

    /**
     * @brief 检查是否包含PCD文件
     * @return true如果包含至少一个PCD文件
     */
    bool HasFiles() const;

    /**
     * @brief 验证激光雷达目录结构
     * @return true如果目录结构有效
     */
    bool Validate() const;
};

/**
 * @brief 完整数据集目录结构
 *
 * 包含相机和激光雷达目录的完整数据集定义，提供验证功能
 */
struct DatasetStructure {
    std::string root_path;              // 数据集根目录
    std::vector<CameraDirectory> cameras; // 相机目录列表
    LidarDirectory lidar;               // 激光雷达目录

    /**
     * @brief 验证数据集结构完整性
     * @return true如果数据集结构完全有效
     */
    bool Validate() const;

    /**
     * @brief 获取详细验证错误信息
     * @return 验证错误信息列表，空列表表示无错误
     */
    std::vector<std::string> GetValidationErrors() const;

    /**
     * @brief 根据相机ID查找相机目录
     * @param camera_id 相机标识符
     * @return 相机目录指针，未找到则返回nullptr
     */
    const CameraDirectory* FindCamera(const std::string& camera_id) const;

    /**
     * @brief 获取数据集摘要信息
     * @return 包含相机数量和PCD文件数量的摘要字符串
     */
    std::string GetSummary() const;

    /**
     * @brief 获取相机和激光雷达文件的时间戳匹配结果
     * @param camera_id 相机ID，为空则匹配所有相机
     * @param max_time_diff_ms 最大允许时间差（毫秒），默认100ms
     * @return 匹配结果列表，按时间戳排序
     */
    std::vector<TimestampMatch> GetTimestampMatches(const std::string& camera_id = "",
                                                   uint64_t max_time_diff_ms = 100) const;
};

} // namespace lidar_camera_calib