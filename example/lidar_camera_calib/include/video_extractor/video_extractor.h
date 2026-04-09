#pragma once

#include "video_types.h"
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @file video_extractor.h
 * @brief 视频帧提取器类定义
 */

namespace lidar_camera_calib {

/**
 * @brief 帧选择策略枚举
 */
enum class FrameSelectionStrategy {
    FIRST_FRAME,      // 第一个有效帧（默认）
    MIDDLE_FRAME,     // 视频中点帧
    SPECIFIC_INDEX,   // 用户指定索引
    SPECIFIC_TIME     // 用户指定时间点
};

/**
 * @brief 提取设置结构
 */
struct ExtractionSettings {
    int jpeg_quality = 95;                    // JPEG压缩质量 (0-100)
    bool preserve_color_space = true;         // 保持原始色彩空间
    bool apply_undistortion = false;          // 可选：应用去畸变

    /**
     * @brief 默认构造函数
     */
    ExtractionSettings() = default;

    /**
     * @brief 验证设置有效性
     * @return true如果所有设置都有效
     */
    bool Validate() const;

    /**
     * @brief 获取设置摘要
     * @return 设置信息的摘要字符串
     */
    std::string GetSummary() const;
};

/**
 * @brief 视频提取错误基类
 */
class VideoExtractionError : public std::exception {
public:
    explicit VideoExtractionError(const std::string& message);
    const char* what() const noexcept override;

protected:
    std::string message_;
};

/**
 * @brief 视频文件未找到异常
 */
class VideoFileNotFound : public VideoExtractionError {
public:
    explicit VideoFileNotFound(const std::string& filename);
};

/**
 * @brief 视频格式不支持异常
 */
class VideoFormatUnsupported : public VideoExtractionError {
public:
    explicit VideoFormatUnsupported(const std::string& filename);
};

/**
 * @brief 视频文件损坏异常
 */
class VideoCorrupted : public VideoExtractionError {
public:
    explicit VideoCorrupted(const std::string& filename);
};

/**
 * @brief 帧索引超出范围异常
 */
class FrameIndexOutOfRange : public VideoExtractionError {
public:
    FrameIndexOutOfRange(int frame_index, int total_frames);
};

/**
 * @brief 输出路径无效异常
 */
class OutputPathInvalid : public VideoExtractionError {
public:
    explicit OutputPathInvalid(const std::string& path);
};

/**
 * @brief 视频帧提取器
 *
 * 负责从视频文件中提取指定帧并保存为图像，支持多种视频格式和输出选项
 * @note 线程安全性：此类不是线程安全的，多线程使用需要外部同步
 * @note 格式支持：依赖OpenCV编译时的codec支持
 */
class VideoExtractor {
public:
    /**
     * @brief 构造函数
     */
    VideoExtractor();

    /**
     * @brief 析构函数
     */
    ~VideoExtractor();

    /**
     * @brief 设置要提取的帧索引
     * @param index 帧索引，默认0（第一帧）
     */
    void SetFrameIndex(int index);

    /**
     * @brief 设置输出图像格式
     * @param format 输出格式 ("jpg", "png", "bmp")
     */
    void SetOutputFormat(const std::string& format);

    /**
     * @brief 设置JPEG质量参数
     * @param quality JPEG质量 (0-100)，仅对JPEG格式有效
     */
    void SetQualityFactor(int quality);

    /**
     * @brief 设置帧选择策略
     * @param strategy 选择策略
     */
    void SetFrameSelectionStrategy(FrameSelectionStrategy strategy);

    /**
     * @brief 设置提取配置
     * @param settings 提取设置
     */
    void SetExtractionSettings(const ExtractionSettings& settings);

    /**
     * @brief 从视频文件提取帧
     * @param video_path 视频文件路径
     * @return ExtractedFrame 提取的帧数据和元信息
     * @throw VideoExtractionError 提取过程中的各种错误
     *
     * @note 合同要求：
     *       - 验证视频文件存在且可读
     *       - 使用OpenCV VideoCapture打开视频
     *       - 在指定索引处提取帧（默认0）
     *       - 对不可读/损坏文件抛出异常
     *       - 返回有效cv::Mat和适当元数据
     */
    ExtractedFrame ExtractFrame(const std::string& video_path) const;

    /**
     * @brief 提取帧并保存到文件
     * @param video_path 视频文件路径
     * @param output_path 输出图像路径
     * @return true如果提取和保存成功
     *
     * @note 合同要求：
     *       - 委托给ExtractFrame()方法
     *       - 使用OpenCV imwrite以指定格式保存
     *       - I/O失败时返回false（不抛异常）
     *       - 保持图像质量设置
     */
    bool ExtractFrameToFile(const std::string& video_path, const std::string& output_path) const;

    /**
     * @brief 获取视频信息
     * @param video_path 视频文件路径
     * @return VideoInfo 视频元数据信息
     *
     * @note 合同要求：
     *       - 提供完整视频元数据
     *       - 优雅处理损坏文件
     *       - 快速完成（最少寻帧）
     */
    VideoInfo GetVideoInfo(const std::string& video_path) const;

    /**
     * @brief 获取当前配置摘要
     * @return 包含帧索引、格式、质量等设置的摘要字符串
     */
    std::string GetConfigSummary() const;

    /**
     * @brief 检查视频格式是否支持
     * @param video_path 视频文件路径
     * @return true如果格式受支持
     */
    bool IsFormatSupported(const std::string& video_path) const;

private:
    int frame_index_;                          // 要提取的帧索引
    std::string output_format_;                // 输出图像格式
    int quality_factor_;                       // JPEG质量因子
    FrameSelectionStrategy strategy_;          // 帧选择策略
    ExtractionSettings settings_;              // 提取设置

    /**
     * @brief 验证视频文件
     * @param video_path 视频文件路径
     * @throw VideoExtractionError 验证失败时抛出
     */
    void ValidateVideoFile(const std::string& video_path) const;

    /**
     * @brief 打开视频捕获器
     * @param video_path 视频文件路径
     * @return 视频捕获器对象
     * @throw VideoExtractionError 打开失败时抛出
     */
    cv::VideoCapture OpenVideoCapture(const std::string& video_path) const;

    /**
     * @brief 根据策略计算实际帧索引
     * @param total_frames 视频总帧数
     * @return 计算后的帧索引
     */
    int CalculateActualFrameIndex(int total_frames) const;

    /**
     * @brief 设置视频捕获器到指定帧
     * @param cap 视频捕获器
     * @param frame_index 目标帧索引
     * @return true如果成功定位到指定帧
     */
    bool SeekToFrame(cv::VideoCapture& cap, int frame_index) const;

    /**
     * @brief 应用图像后处理
     * @param image 原始图像
     * @return 后处理后的图像
     */
    cv::Mat ApplyPostProcessing(const cv::Mat& image) const;

    /**
     * @brief 获取输出编码参数
     * @return OpenCV编码参数向量
     */
    std::vector<int> GetOutputEncodingParams() const;

    /**
     * @brief 验证输出路径
     * @param output_path 输出路径
     * @throw OutputPathInvalid 路径无效时抛出
     */
    void ValidateOutputPath(const std::string& output_path) const;

    /**
     * @brief 处理帧索引超出范围的情况
     * @param requested_index 请求的帧索引
     * @param total_frames 视频总帧数
     * @return 调整后的有效帧索引
     */
    int HandleFrameIndexOutOfRange(int requested_index, int total_frames) const;
};

} // namespace lidar_camera_calib