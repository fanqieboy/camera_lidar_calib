#include "video_extractor.h"
#include <filesystem>
#include <algorithm>
#include <sstream>

/**
 * @file video_extractor.cpp
 * @brief 视频帧提取器类实现
 */

namespace lidar_camera_calib {

// ExtractionSettings implementation
bool ExtractionSettings::Validate() const { return jpeg_quality >= 0 && jpeg_quality <= 100; }

std::string ExtractionSettings::GetSummary() const {
  std::ostringstream oss;
  oss << "ExtractionSettings: JPEG Quality=" << jpeg_quality
      << ", Preserve Color Space=" << (preserve_color_space ? "Yes" : "No")
      << ", Apply Undistortion=" << (apply_undistortion ? "Yes" : "No");
  return oss.str();
}

// Exception implementations
VideoExtractionError::VideoExtractionError(const std::string& message) : message_(message) {}

const char* VideoExtractionError::what() const noexcept { return message_.c_str(); }

VideoFileNotFound::VideoFileNotFound(const std::string& filename)
    : VideoExtractionError("Video file not found: " + filename) {}

VideoFormatUnsupported::VideoFormatUnsupported(const std::string& filename)
    : VideoExtractionError("Video format unsupported: " + filename) {}

VideoCorrupted::VideoCorrupted(const std::string& filename)
    : VideoExtractionError("Video file corrupted: " + filename) {}

FrameIndexOutOfRange::FrameIndexOutOfRange(int frame_index, int total_frames)
    : VideoExtractionError("Frame index out of range: " + std::to_string(frame_index) +
                           " (total frames: " + std::to_string(total_frames) + ")") {}

OutputPathInvalid::OutputPathInvalid(const std::string& path) : VideoExtractionError("Output path invalid: " + path) {}

// VideoExtractor implementation
VideoExtractor::VideoExtractor()
    : frame_index_(0), output_format_("jpg"), quality_factor_(95), strategy_(FrameSelectionStrategy::FIRST_FRAME) {}

VideoExtractor::~VideoExtractor() = default;

void VideoExtractor::SetFrameIndex(int index) { frame_index_ = std::max(0, index); }

void VideoExtractor::SetOutputFormat(const std::string& format) {
  std::string lower_format = format;
  std::transform(lower_format.begin(), lower_format.end(), lower_format.begin(), ::tolower);

  if (lower_format == "jpg" || lower_format == "jpeg" || lower_format == "png" || lower_format == "bmp") {
    output_format_ = lower_format;
  } else {
    throw std::invalid_argument("Unsupported output format: " + format);
  }
}

void VideoExtractor::SetQualityFactor(int quality) { quality_factor_ = std::clamp(quality, 0, 100); }

void VideoExtractor::SetFrameSelectionStrategy(FrameSelectionStrategy strategy) { strategy_ = strategy; }

void VideoExtractor::SetExtractionSettings(const ExtractionSettings& settings) {
  if (!settings.Validate()) {
    throw std::invalid_argument("Invalid extraction settings");
  }
  settings_ = settings;
}

ExtractedFrame VideoExtractor::ExtractFrame(const std::string& video_path) const {
  // Validate video file first
  ValidateVideoFile(video_path);

  // Open video capture
  cv::VideoCapture cap = OpenVideoCapture(video_path);

  // Get video information
  int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
  double fps = cap.get(cv::CAP_PROP_FPS);

  if (total_frames <= 0 || fps <= 0) {
    throw VideoCorrupted(video_path);
  }

  // Calculate actual frame index based on strategy
  int actual_frame_index = CalculateActualFrameIndex(total_frames);

  // Handle frame index out of range
  actual_frame_index = HandleFrameIndexOutOfRange(actual_frame_index, total_frames);

  // Seek to the desired frame
  if (!SeekToFrame(cap, actual_frame_index)) {
    throw VideoExtractionError("Failed to seek to frame " + std::to_string(actual_frame_index));
  }

  // Extract the frame
  cv::Mat frame;
  if (!cap.read(frame) || frame.empty()) {
    throw VideoExtractionError("Failed to read frame at index " + std::to_string(actual_frame_index));
  }

  // Apply post-processing if needed
  cv::Mat processed_frame = ApplyPostProcessing(frame);

  // Create ExtractedFrame object
  ExtractedFrame result;
  result.image = processed_frame;
  result.source_video = video_path;
  result.frame_index = actual_frame_index;
  result.timestamp = actual_frame_index / fps;

  return result;
}

bool VideoExtractor::ExtractFrameToFile(const std::string& video_path, const std::string& output_path) const {
  try {
    // Validate output path
    ValidateOutputPath(output_path);

    // Extract frame
    ExtractedFrame extracted = ExtractFrame(video_path);

    // Save to file
    std::vector<int> encoding_params = GetOutputEncodingParams();
    bool success = cv::imwrite(output_path, extracted.image, encoding_params);

    return success;
  } catch (const std::exception&) {
    // Return false for I/O failures as per contract
    return false;
  }
}

VideoInfo VideoExtractor::GetVideoInfo(const std::string& video_path) const {
  VideoInfo info;

  try {
    // Validate file existence
    if (!std::filesystem::exists(video_path)) {
      info.is_valid = false;
      return info;
    }

    // Open video capture
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
      info.is_valid = false;
      return info;
    }

    // Read video properties
    info.total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    info.fps = cap.get(cv::CAP_PROP_FPS);
    info.frame_size.width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    info.frame_size.height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    info.codec_fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));

    // Calculate duration
    if (info.fps > 0) {
      info.duration_seconds = info.total_frames / info.fps;
    } else {
      info.duration_seconds = 0;
    }

    // Validate completeness
    info.is_valid = info.Validate();

  } catch (const std::exception&) {
    info.is_valid = false;
  }

  return info;
}

std::string VideoExtractor::GetConfigSummary() const {
  std::ostringstream oss;
  oss << "VideoExtractor Config:\n";
  oss << "  Frame Index: " << frame_index_ << "\n";
  oss << "  Output Format: " << output_format_ << "\n";
  oss << "  Quality Factor: " << quality_factor_ << "\n";
  oss << "  Strategy: ";
  switch (strategy_) {
    case FrameSelectionStrategy::FIRST_FRAME:
      oss << "First Frame";
      break;
    case FrameSelectionStrategy::MIDDLE_FRAME:
      oss << "Middle Frame";
      break;
    case FrameSelectionStrategy::SPECIFIC_INDEX:
      oss << "Specific Index";
      break;
    case FrameSelectionStrategy::SPECIFIC_TIME:
      oss << "Specific Time";
      break;
  }
  oss << "\n  " << settings_.GetSummary();
  return oss.str();
}

bool VideoExtractor::IsFormatSupported(const std::string& video_path) const {
  try {
    cv::VideoCapture cap(video_path);
    return cap.isOpened();
  } catch (const std::exception&) {
    return false;
  }
}

void VideoExtractor::ValidateVideoFile(const std::string& video_path) const {
  if (video_path.empty()) {
    throw std::invalid_argument("Video path cannot be empty");
  }

  if (!std::filesystem::exists(video_path)) {
    throw VideoFileNotFound(video_path);
  }

  if (!std::filesystem::is_regular_file(video_path)) {
    throw VideoFileNotFound(video_path);
  }

  // Check file extension for basic format validation
  std::string extension = std::filesystem::path(video_path).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

  std::vector<std::string> supported_extensions = {".mp4", ".avi", ".mov", ".mkv"};
  bool format_supported =
      std::find(supported_extensions.begin(), supported_extensions.end(), extension) != supported_extensions.end();

  if (!format_supported) {
    throw VideoFormatUnsupported(video_path);
  }
}

cv::VideoCapture VideoExtractor::OpenVideoCapture(const std::string& video_path) const {
  cv::VideoCapture cap(video_path);

  if (!cap.isOpened()) {
    throw VideoCorrupted(video_path);
  }

  return cap;
}

int VideoExtractor::CalculateActualFrameIndex(int total_frames) const {
  switch (strategy_) {
    case FrameSelectionStrategy::FIRST_FRAME:
      return 0;
    case FrameSelectionStrategy::MIDDLE_FRAME:
      return total_frames / 2;
    case FrameSelectionStrategy::SPECIFIC_INDEX:
      return frame_index_;
    case FrameSelectionStrategy::SPECIFIC_TIME:
      // For time-based selection, use frame_index_ as time in seconds
      // This is a simplified implementation
      return frame_index_;
    default:
      return 0;
  }
}

bool VideoExtractor::SeekToFrame(cv::VideoCapture& cap, int frame_index) const {
  if (frame_index < 0) {
    return false;
  }

  return cap.set(cv::CAP_PROP_POS_FRAMES, frame_index);
}

cv::Mat VideoExtractor::ApplyPostProcessing(const cv::Mat& image) const {
  cv::Mat result = image.clone();

  // Apply color space preservation if needed
  if (settings_.preserve_color_space) {
    // Keep original color space - no conversion needed
  }

  // Apply undistortion if configured
  if (settings_.apply_undistortion) {
    // This would require camera calibration parameters
    // For now, just return the original image
    // In a real implementation, you would apply cv::undistort() here
  }

  return result;
}

std::vector<int> VideoExtractor::GetOutputEncodingParams() const {
  std::vector<int> params;

  if (output_format_ == "jpg" || output_format_ == "jpeg") {
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(settings_.jpeg_quality);
  } else if (output_format_ == "png") {
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(9);  // Maximum compression
  }
  // For BMP, no special parameters needed

  return params;
}

void VideoExtractor::ValidateOutputPath(const std::string& output_path) const {
  if (output_path.empty()) {
    throw OutputPathInvalid("Output path cannot be empty");
  }

  // Check parent directory exists
  std::filesystem::path path(output_path);
  if (!path.parent_path().empty() && !std::filesystem::exists(path.parent_path())) {
    throw OutputPathInvalid("Parent directory does not exist: " + path.parent_path().string());
  }

  // Check if we can write to the location
  if (std::filesystem::exists(output_path) && !std::filesystem::is_regular_file(output_path)) {
    throw OutputPathInvalid("Output path exists but is not a file: " + output_path);
  }
}

int VideoExtractor::HandleFrameIndexOutOfRange(int requested_index, int total_frames) const {
  if (requested_index < 0) {
    // Use first frame
    return 0;
  }

  if (requested_index >= total_frames) {
    // Use last frame
    return total_frames - 1;
  }

  return requested_index;
}

}  // namespace lidar_camera_calib