#pragma once

#include "dataset_structure.h"
#include <string>
#include <vector>

/**
 * @file dataset_loader.h
 */

namespace lidar_camera_calib {

/**
 * @brief 验证结果结构
 *
 * 包含验证状态和详细的错误/警告信息
 */
struct ValidationResult {
    bool success;                        // 验证是否成功
    std::vector<std::string> errors;     // 错误信息列表
    std::vector<std::string> warnings;   // 警告信息列表

    /**
     * @brief 默认构造函数，初始化为失败状态
     */
    ValidationResult();

    /**
     * @brief 添加错误信息
     * @param error 错误描述
     */
    void AddError(const std::string& error);

    /**
     * @brief 添加警告信息
     * @param warning 警告描述
     */
    void AddWarning(const std::string& warning);

    /**
     * @brief 检查是否有错误
     * @return true如果存在错误信息
     */
    bool HasErrors() const;

    /**
     * @brief 获取所有信息的摘要
     * @return 包含错误和警告数量的摘要字符串
     */
    std::string GetSummary() const;
};

/**
 * @brief 数据集验证异常基类
 */
class DatasetValidationError : public std::exception {
public:
    explicit DatasetValidationError(const std::string& message);
    const char* what() const noexcept override;

protected:
    std::string message_;
};

/**
 * @brief 相机目录未找到异常
 */
class CameraDirectoryNotFound : public DatasetValidationError {
public:
    explicit CameraDirectoryNotFound(const std::string& camera_id);
};

/**
 * @brief 激光雷达目录未找到异常
 */
class LidarDirectoryNotFound : public DatasetValidationError {
public:
    explicit LidarDirectoryNotFound(const std::string& directory_path);
};

/**
 * @brief PCD文件不足异常
 */
class InsufficientPcdFiles : public DatasetValidationError {
public:
    InsufficientPcdFiles(int found_count, int required_count);
};

/**
 * @brief 数据集结构无效异常
 */
class InvalidDatasetStructure : public DatasetValidationError {
public:
    explicit InvalidDatasetStructure(const std::string& details);
};

/**
 * @brief 数据集加载器
 *
 * 负责验证和加载特定格式的数据集目录结构，支持cameras/camera_X和lidar/目录
 * @note 线程安全性：此类不是线程安全的，多线程使用需要外部同步
 */
class DatasetLoader {
public:
    /**
     * @brief 构造函数
     * @param root_path 数据集根目录路径
     * @throw std::invalid_argument 如果root_path为空或不存在
     */
    explicit DatasetLoader(const std::string& root_path);

    /**
     * @brief 设置相机ID过滤
     * @param camera_id 要加载的特定相机ID（如"camera_0"），空字符串表示加载所有相机
     */
    void SetCameraId(const std::string& camera_id);

    /**
     * @brief 设置PCD文件数量限制
     * @param max_count 最大PCD文件数量，-1表示不限制
     */
    void SetMaxPcdFiles(int max_count);

    /**
     * @brief 验证数据集目录结构
     * @return ValidationResult 验证结果，包含错误和警告信息
     *
     * @note 验证规则：
     *       - 检查目录结构存在性
     *       - 验证至少一个相机目录
     *       - 验证至少一个PCD文件在lidar/目录
     *       - 返回详细的错误信息
     */
    ValidationResult Validate() const;

    /**
     * @brief 加载数据集结构
     * @return DatasetStructure 完整的数据集结构
     * @throw DatasetValidationError 如果验证失败
     *
     * @note 执行流程：
     *       - 首先调用Validate()
     *       - 如果验证失败则抛出异常
     *       - 返回完整填充的DatasetStructure
     *       - PCD文件按文件名排序
     */
    DatasetStructure LoadDataset() const;

    /**
     * @brief 获取当前配置摘要
     * @return 包含根路径、相机ID过滤和文件限制的摘要字符串
     */
    std::string GetConfigSummary() const;

    /**
     * @brief 设置时间戳匹配的最大时间差
     * @param max_time_diff_ms 最大时间差（毫秒），默认100ms
     */
    void SetMaxTimeDifference(uint64_t max_time_diff_ms);

    /**
     * @brief 检查文件名是否符合时间戳命名格式
     * @param filename 文件名（不含路径）
     * @return true如果符合YYYYMMDDHHMMSS_NNNNNNNNN格式
     */
    static bool IsTimestampFormat(const std::string& filename);

    /**
     * @brief 解析文件名中的时间戳信息（公开接口，用于测试）
     * @param filename 文件名（不含路径）
     * @param full_path 完整文件路径
     * @return 时间戳信息，解析失败则返回无效的TimestampInfo
     */
    static TimestampInfo ParseTimestampPublic(const std::string& filename, const std::string& full_path);

private:
    std::string root_path_;      // 数据集根目录
    std::string camera_id_;      // 相机ID过滤（空表示所有）
    int max_pcd_files_;          // PCD文件数量限制
    uint64_t max_time_diff_ms_;  // 时间戳匹配最大时间差（毫秒）

    /**
     * @brief 扫描相机目录
     * @return 相机目录列表
     */
    std::vector<CameraDirectory> ScanCameraDirectories() const;

    /**
     * @brief 扫描激光雷达目录
     * @return 激光雷达目录结构
     */
    LidarDirectory ScanLidarDirectory() const;

    /**
     * @brief 验证目录存在性
     * @param directory_path 目录路径
     * @return true如果目录存在且可访问
     */
    bool DirectoryExists(const std::string& directory_path) const;

    /**
     * @brief 获取目录中的文件列表
     * @param directory_path 目录路径
     * @param extension 文件扩展名过滤（如".mp4"）
     * @return 匹配的文件路径列表（已排序）
     */
    std::vector<std::string> GetFilesInDirectory(const std::string& directory_path,
                                                 const std::string& extension) const;

    /**
     * @brief 解析文件名中的时间戳信息
     * @param filename 文件名（不含路径）
     * @param full_path 完整文件路径
     * @return 时间戳信息，解析失败则返回无效的TimestampInfo
     */
    static TimestampInfo ParseTimestamp(const std::string& filename, const std::string& full_path);

    /**
     * @brief 将时间戳字符串转换为纳秒级时间戳
     * @param date_part 日期部分 (YYYYMMDDHHMMSS)
     * @param nano_part 纳秒部分 (NNNNNNNNN)
     * @return 纳秒级时间戳，转换失败返回0
     */
    static uint64_t ConvertToNanoseconds(const std::string& date_part, const std::string& nano_part);
};

} // namespace lidar_camera_calib