#pragma once

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @file video_types.h
 * @brief 视频处理相关数据类型定义
 */

namespace lidar_camera_calib {

/**
 * @brief 视频文件基本信息
 *
 * 包含视频文件的元数据信息，用于视频处理前的检查和配置
 */
struct VideoInfo {
    int total_frames;           // 视频总帧数
    double fps;                 // 帧率
    double duration_seconds;    // 视频时长（秒）
    cv::Size frame_size;        // 帧尺寸
    int codec_fourcc;           // 编解码器四字符码
    bool is_valid;              // 视频文件是否有效

    /**
     * @brief 默认构造函数，初始化为无效状态
     */
    VideoInfo();

    /**
     * @brief 获取编解码器名称
     * @return 四字符编解码器名称字符串
     */
    std::string GetCodecName() const;

    /**
     * @brief 检查视频是否支持帧提取
     * @return true如果视频格式支持提取操作
     */
    bool IsSupportedForExtraction() const;

    /**
     * @brief 获取视频信息摘要
     * @return 包含关键参数的摘要字符串
     */
    std::string GetSummary() const;

    /**
     * @brief 验证视频信息完整性
     * @return true如果所有信息都有效
     */
    bool Validate() const;

    /**
     * @brief 检查指定帧索引是否有效
     * @param frame_index 帧索引
     * @return true如果帧索引在有效范围内
     */
    bool IsFrameIndexValid(int frame_index) const;
};

/**
 * @brief 从视频提取的图像帧
 *
 * 包含提取的图像数据和相关元信息，用于后续的标定处理
 */
struct ExtractedFrame {
    cv::Mat image;                      // 提取的图像数据
    std::string source_video;           // 源视频文件路径
    double timestamp;                   // 帧时间戳（秒）
    int frame_index;                    // 帧索引

    /**
     * @brief 默认构造函数
     */
    ExtractedFrame();

    /**
     * @brief 构造函数，初始化基本信息
     * @param video_path 源视频路径
     * @param frame_idx 帧索引
     * @param time_stamp 时间戳
     */
    ExtractedFrame(const std::string& video_path, int frame_idx, double time_stamp);

    /**
     * @brief 检查提取的帧是否有效
     * @return true如果图像数据和元信息都有效
     *
     * @note 检查项目包括：
     *       - 图像不为空
     *       - 源视频路径非空
     *       - 时间戳非负
     */
    bool IsValid() const;

    /**
     * @brief 获取图像尺寸
     * @return 图像的宽度和高度
     */
    cv::Size GetDimensions() const;

    /**
     * @brief 获取图像通道数
     * @return 图像通道数（通常为1或3）
     */
    int GetChannels() const;

    /**
     * @brief 获取图像数据类型信息
     * @return OpenCV数据类型标识符
     */
    int GetDepth() const;

    /**
     * @brief 保存提取的帧到文件
     * @param output_path 输出文件路径
     * @param quality JPEG质量参数（0-100），默认95
     * @return true如果保存成功
     */
    bool SaveToFile(const std::string& output_path, int quality = 95) const;

    /**
     * @brief 获取提取帧的摘要信息
     * @return 包含图像基本信息的摘要字符串
     */
    std::string GetSummary() const;

    /**
     * @brief 创建图像的深拷贝
     * @return 新的ExtractedFrame对象，包含图像数据的深拷贝
     */
    ExtractedFrame Clone() const;
};

} // namespace lidar_camera_calib