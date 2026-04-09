#include "video_types.h"
#include <sstream>
#include <filesystem>

namespace lidar_camera_calib {

// VideoInfo 实现

VideoInfo::VideoInfo()
    : total_frames(0), fps(0.0), duration_seconds(0.0), frame_size(0, 0), codec_fourcc(0), is_valid(false) {}

std::string VideoInfo::GetCodecName() const {
  if (codec_fourcc == 0) {
    return "Unknown";
  }

  char fourcc_str[5] = {0};
  fourcc_str[0] = static_cast<char>(codec_fourcc & 0xff);
  fourcc_str[1] = static_cast<char>((codec_fourcc >> 8) & 0xff);
  fourcc_str[2] = static_cast<char>((codec_fourcc >> 16) & 0xff);
  fourcc_str[3] = static_cast<char>((codec_fourcc >> 24) & 0xff);

  return std::string(fourcc_str);
}

bool VideoInfo::IsSupportedForExtraction() const {
  if (!is_valid || total_frames <= 0) {
    return false;
  }

  // 检查基本参数有效性
  if (frame_size.width <= 0 || frame_size.height <= 0) {
    return false;
  }

  return true;
}

std::string VideoInfo::GetSummary() const {
  std::ostringstream oss;
  oss << "Video Info Summary:\n";
  oss << "  Valid: " << (is_valid ? "Yes" : "No") << "\n";
  if (is_valid) {
    oss << "  Resolution: " << frame_size.width << "x" << frame_size.height << "\n";
    oss << "  Total Frames: " << total_frames << "\n";
    oss << "  FPS: " << fps << "\n";
    oss << "  Duration: " << duration_seconds << "s\n";
    oss << "  Codec: " << GetCodecName() << "\n";
  }
  return oss.str();
}

bool VideoInfo::Validate() const {
  if (!is_valid) {
    return false;
  }

  return total_frames > 0 && fps > 0.0 && duration_seconds > 0.0 && frame_size.width > 0 && frame_size.height > 0;
}

bool VideoInfo::IsFrameIndexValid(int frame_index) const {
  return is_valid && frame_index >= 0 && frame_index < total_frames;
}

// ExtractedFrame 实现

ExtractedFrame::ExtractedFrame() : timestamp(0.0), frame_index(0) {}

ExtractedFrame::ExtractedFrame(const std::string& video_path, int frame_idx, double time_stamp)
    : source_video(video_path), timestamp(time_stamp), frame_index(frame_idx) {}

bool ExtractedFrame::IsValid() const {
  // 检查图像数据
  if (image.empty()) {
    return false;
  }

  // 检查源视频路径
  if (source_video.empty()) {
    return false;
  }

  // 检查时间戳
  if (timestamp < 0.0) {
    return false;
  }

  // 检查帧索引
  if (frame_index < 0) {
    return false;
  }

  return true;
}

cv::Size ExtractedFrame::GetDimensions() const {
  return image.empty() ? cv::Size(0, 0) : cv::Size(image.cols, image.rows);
}

int ExtractedFrame::GetChannels() const { return image.empty() ? 0 : image.channels(); }

int ExtractedFrame::GetDepth() const { return image.empty() ? -1 : image.depth(); }

bool ExtractedFrame::SaveToFile(const std::string& output_path, int quality) const {
  if (image.empty()) {
    return false;
  }

  try {
    std::vector<int> params;

    // 根据文件扩展名设置参数
    std::string ext = std::filesystem::path(output_path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == ".jpg" || ext == ".jpeg") {
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(std::max(0, std::min(100, quality)));
    } else if (ext == ".png") {
      params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      params.push_back(9);  // 最大压缩
    }

    return cv::imwrite(output_path, image, params);
  } catch (const cv::Exception&) {
    return false;
  }
}

std::string ExtractedFrame::GetSummary() const {
  std::ostringstream oss;
  oss << "Extracted Frame Summary:\n";
  oss << "  Valid: " << (IsValid() ? "Yes" : "No") << "\n";
  if (IsValid()) {
    oss << "  Source: " << source_video << "\n";
    oss << "  Frame Index: " << frame_index << "\n";
    oss << "  Timestamp: " << timestamp << "s\n";
    oss << "  Dimensions: " << GetDimensions().width << "x" << GetDimensions().height << "\n";
    oss << "  Channels: " << GetChannels() << "\n";
  }
  return oss.str();
}

ExtractedFrame ExtractedFrame::Clone() const {
  ExtractedFrame cloned(source_video, frame_index, timestamp);
  if (!image.empty()) {
    cloned.image = image.clone();
  }
  return cloned;
}

}  // namespace lidar_camera_calib