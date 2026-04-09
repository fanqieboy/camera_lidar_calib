/**
 * @file cli.cpp
 * @brief 命令行接口模块实现 - 提供独立的命令行参数解析和处理功能
 *
 * 该文件是Cli类的具体实现，包括所有公共和私有方法的实现。
 */

#include "cli.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>

#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

// 静态常量定义
const std::vector<std::string> Cli::kSupportedOutputFormats = {"txt", "yaml", "json", "xml", "csv"};

const std::string Cli::kColorRed = "\033[31m";
const std::string Cli::kColorGreen = "\033[32m";
const std::string Cli::kColorYellow = "\033[33m";
const std::string Cli::kColorBlue = "\033[34m";
const std::string Cli::kColorMagenta = "\033[35m";
const std::string Cli::kColorCyan = "\033[36m";
const std::string Cli::kColorReset = "\033[0m";

const std::string Cli::kVersionMajor = "2";
const std::string Cli::kVersionMinor = "0";
const std::string Cli::kVersionPatch = "0";
const std::string Cli::kBuildDate = __DATE__;

Cli::Cli() { RegisterOptions(); }

Cli::~Cli() {}

CliOptions Cli::ParseArguments(int argc, char* argv[]) {
  CliOptions options;

  if (argc <= 1) {
    options.help = true;
    return options;
  }

  try {
    for (int i = 1; i < argc;) {
      std::string arg = argv[i];
      std::string next_arg = (i + 1 < argc) ? argv[i + 1] : "";

      int consumed = ParseArgument(arg, next_arg, options);
      if (consumed == 0) {
        ReportError("Unknown argument: " + arg);
        options.help = true;
        return options;
      }

      i += consumed;
    }

    // 验证选项
    if (!options.help && !options.version && !ValidateOptions(options)) {
      options.help = true;
    }
  } catch (const std::exception& e) {
    ReportError("Error parsing arguments: " + std::string(e.what()));
    options.help = true;
  }

  return options;
}

void Cli::ShowHelp(const std::string& program_name) const {
  std::cout << ColorText("Extrinsic-Calib Standalone", kColorCyan) << " - LiDAR-Camera Calibration Tool\n";
  std::cout << "Version " << kVersionMajor << "." << kVersionMinor << "." << kVersionPatch << " (Built on "
            << kBuildDate << ")\n\n";

  std::cout << ColorText("USAGE:", kColorYellow) << "\n";
  std::cout << "  " << program_name << " [OPTIONS]\n\n";

  std::cout << ColorText("MODES:", kColorYellow) << "\n";
  std::cout << "  --single-scene, -s          Single scene calibration mode (default)\n";
  std::cout << "  --multi-scene, -m           Multi-scene calibration mode\n";
  std::cout << "  --process-only, -p          Process data without calibration\n";
  std::cout << "  --batch, -b                 Batch processing mode\n";
  std::cout << "  --interactive, -i           Interactive mode\n\n";

  std::cout << ColorText("INPUT OPTIONS:", kColorYellow) << "\n";
  std::cout << "  --config, -c FILE           Configuration file path\n";
  std::cout << "  --image IMAGE               Single image file path\n";
  std::cout << "  --pointcloud PC             Single point cloud file path\n";
  std::cout << "  --data-dir DIR              Data directory containing images and point clouds\n";
  std::cout << "  --record-file FILE          Multi-scene data record file\n";

  std::cout << ColorText("OUTPUT OPTIONS:", kColorYellow) << "\n";
  std::cout << "  --prefix PREFIX             Output file prefix (default: calib_result)\n";
  std::cout << "  --format FORMAT             Output format(s): txt,yaml,json,xml,csv\n\n";

  std::cout << ColorText("PROCESSING OPTIONS:", kColorYellow) << "\n";
  std::cout << "  --scenes SELECTION          Scene selection (e.g., \"1,3,5-8,10\")\n";
  std::cout << "  --max-scenes N              Maximum number of scenes to process\n";
  std::cout << "  --rmse-threshold THRESH     RMSE threshold for validation\n\n";

  std::cout << ColorText("UNDISTORTION OPTIONS:", kColorYellow) << "\n";
  std::cout << "  --undistort, -u             Enable undistortion preprocessing\n";
  std::cout << "  --distortion-model MODEL    Distortion model (pinhole-radtan, pinhole-equi, omni-radtan)\n";
  std::cout << "  --alpha VALUE               Undistortion scaling factor (0.0-1.0, default: 1.0)\n";
  std::cout << "  --output-dataset DIR        Output dataset directory for undistorted data\n";
  std::cout << "  --preserve-original         Preserve original data (default: true)\n\n";

  std::cout << ColorText("GENERAL OPTIONS:", kColorYellow) << "\n";
  std::cout << "  --verbose                   Enable verbose output\n";
  std::cout << "  --quiet                     Suppress non-error output\n";
  std::cout << "  --dry-run                   Show what would be done without executing\n";
  std::cout << "  --help, -h                  Show this help message\n";
  std::cout << "  --version                   Show version information\n\n";

  ShowExamples();
}

void Cli::ShowVersion() const {
  std::cout << "LiDAR-Camera-Calib Standalone " << kVersionMajor << "." << kVersionMinor << "." << kVersionPatch << std::endl;
  std::cout << "Built on " << kBuildDate << std::endl;
  std::cout << "ROS-independent LiDAR-Camera extrinsic calibration tool" << std::endl;
}

void Cli::ShowExamples() const {
  std::cout << ColorText("EXAMPLES:", kColorGreen) << "\n";
  std::cout << "  # Single scene calibration with default config\n";
  std::cout << "  lidar-camera-calib --image camera.png --pointcloud lidar.pcd\n\n";

  std::cout << "  # Single scene with custom config\n";
  std::cout << "  lidar-camera-calib -c config.yaml --image cam.png --pointcloud pc.pcd\n\n";

  std::cout << "  # Multi-scene calibration from data directory\n";
  std::cout << "  lidar-camera-calib --multi-scene --data-dir ./calib_data --format yaml,json\n\n";

  std::cout << "  # Multi-scene with specific scene selection\n";
  std::cout << "  lidar-camera-calib -m --record-file centers.txt --scenes \"1,3,5-8\"\n\n";

  std::cout << "  # Batch processing with custom output format\n";
  std::cout << "  lidar-camera-calib --batch --data-dir ./batch_data --format yaml,json\n\n";

  std::cout << "  # Process only (no calibration) mode\n";
  std::cout << "  lidar-camera-calib --process-only --data-dir ./test_data\n\n";

  std::cout << "  # Undistortion preprocessing for fisheye camera\n";
  std::cout << "  lidar-camera-calib --undistort --distortion-model pinhole-equi --alpha 0.8\n\n";

  std::cout << "  # Combined undistortion and calibration\n";
  std::cout << "  lidar-camera-calib -c config.yaml --undistort --output-dataset ./undistorted_data\n\n";
}

bool Cli::ValidateOptions(const CliOptions& options) {
  bool valid = true;

  // 注意：由于数据集目录 (dataset_dir) 现在通过配置文件提供，
  // 完整的输入验证移到主程序中进行，这里只进行基本的命令行参数验证

  // 检查文件存在性
  if (!options.config_file.empty() && !FileExists(options.config_file)) {
    ReportError("Configuration file not found: " + options.config_file);
    valid = false;
  }

  if (!options.image_path.empty() && !FileExists(options.image_path)) {
    ReportError("Image file not found: " + options.image_path);
    valid = false;
  }

  if (!options.pointcloud_path.empty() && !FileExists(options.pointcloud_path)) {
    ReportError("Point cloud file not found: " + options.pointcloud_path);
    valid = false;
  }

  if (!options.data_directory.empty() && !DirectoryExists(options.data_directory)) {
    ReportError("Data directory not found: " + options.data_directory);
    valid = false;
  }

  if (!options.record_file.empty() && !FileExists(options.record_file)) {
    ReportError("Record file not found: " + options.record_file);
    valid = false;
  }

  // 检查输出格式
  for (const auto& format : options.output_formats) {
    if (!IsValidOutputFormat(format)) {
      ReportError("Unsupported output format: " + format);
      valid = false;
    }
  }

  // 检查冲突选项
  if (options.verbose && options.quiet) {
    ReportError("Cannot specify both --verbose and --quiet");
    valid = false;
  }

  // 检查数据输入冲突
  int input_count = 0;
  if (!options.data_directory.empty()) input_count++;
  if (!options.image_path.empty() && !options.pointcloud_path.empty()) input_count++;
  if (!options.record_file.empty()) input_count++;

  if (input_count > 1) {
    ReportError("Cannot specify multiple data input methods simultaneously");
    valid = false;
  }

  // 检查数值范围
  if (options.max_scenes != -1 && options.max_scenes <= 0) {
    ReportError("Max scenes must be a positive number");
    valid = false;
  }

  if (options.rmse_threshold != -1.0 && options.rmse_threshold <= 0.0) {
    ReportError("RMSE threshold must be a positive number");
    valid = false;
  }

  // 检查去畸变参数
  if (options.undistort_preprocessing) {
    if (options.alpha < 0.0 || options.alpha > 1.0) {
      ReportError("Alpha value must be between 0.0 and 1.0");
      valid = false;
    }

    if (!options.distortion_model.empty() && options.distortion_model != "pinhole-radtan" &&
        options.distortion_model != "pinhole-equi" && options.distortion_model != "omni-radtan") {
      ReportError("Unsupported distortion model: " + options.distortion_model);
      valid = false;
    }
  }

  return valid;
}

void Cli::ShowOptionsSummary(const CliOptions& options) const {
  std::cout << ColorText("Configuration Summary:", kColorCyan) << std::endl;

  // 模式
  std::string mode_str;
  switch (options.mode) {
    case CliOptions::Mode::kSingleScene:
      mode_str = "Single Scene";
      break;
    case CliOptions::Mode::kMultiScene:
      mode_str = "Multi Scene";
      break;
    case CliOptions::Mode::kProcessOnly:
      mode_str = "Process Only";
      break;
    case CliOptions::Mode::kBatchProcess:
      mode_str = "Batch Process";
      break;
    case CliOptions::Mode::kInteractive:
      mode_str = "Interactive";
      break;
  }
  std::cout << "  Mode: " << ColorText(mode_str, kColorGreen) << std::endl;

  // 输入文件
  if (!options.config_file.empty()) {
    std::cout << "  Config: " << options.config_file << std::endl;
  }
  if (!options.image_path.empty()) {
    std::cout << "  Image: " << options.image_path << std::endl;
  }
  if (!options.pointcloud_path.empty()) {
    std::cout << "  Point Cloud: " << options.pointcloud_path << std::endl;
  }
  if (!options.data_directory.empty()) {
    std::cout << "  Data Directory: " << options.data_directory << std::endl;
  }
  if (!options.record_file.empty()) {
    std::cout << "  Record File: " << options.record_file << std::endl;
  }

  // 输出设置
  std::cout << "  Output Prefix: " << options.output_prefix << std::endl;

  if (!options.output_formats.empty()) {
    std::cout << "  Output Formats: ";
    for (size_t i = 0; i < options.output_formats.size(); ++i) {
      std::cout << options.output_formats[i];
      if (i < options.output_formats.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
  }

  // 其他选项
  std::vector<std::string> flags;
  if (options.verbose) flags.push_back("Verbose");
  if (options.quiet) flags.push_back("Quiet");
  if (options.dry_run) flags.push_back("Dry Run");

  if (!flags.empty()) {
    std::cout << "  Options: ";
    for (size_t i = 0; i < flags.size(); ++i) {
      std::cout << flags[i];
      if (i < flags.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
  }

  // 多场景特定选项
  if (options.mode == CliOptions::Mode::kMultiScene) {
    if (!options.selected_scenes.empty()) {
      std::cout << "  Selected Scenes: ";
      for (size_t i = 0; i < options.selected_scenes.size(); ++i) {
        std::cout << options.selected_scenes[i];
        if (i < options.selected_scenes.size() - 1) std::cout << ", ";
      }
      std::cout << std::endl;
    }
    if (options.max_scenes != -1) {
      std::cout << "  Max Scenes: " << options.max_scenes << std::endl;
    }
  }

  if (options.rmse_threshold != -1.0) {
    std::cout << "  RMSE Threshold: " << options.rmse_threshold << std::endl;
  }

  // 去畸变选项
  if (options.undistort_preprocessing) {
    std::cout << "  " << ColorText("Undistortion Preprocessing:", kColorMagenta) << " Enabled\n";
    std::cout << "    Distortion Model: " << options.distortion_model << std::endl;
    std::cout << "    Alpha Factor: " << options.alpha << std::endl;
    if (!options.output_dataset_dir.empty()) {
      std::cout << "    Output Dataset: " << options.output_dataset_dir << std::endl;
    }
    std::cout << "    Preserve Original: " << (options.preserve_original ? "Yes" : "No") << std::endl;
  }

  std::cout << std::endl;
}

std::vector<int> Cli::ParseSceneSelection(const std::string& selection_str) {
  std::vector<int> indices;
  std::stringstream ss(selection_str);
  std::string item;

  while (std::getline(ss, item, ',')) {
    // 去除空格
    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());

    if (item.empty()) continue;

    try {
      // 检查是否为范围（如"1-5"）
      size_t dash_pos = item.find('-');
      if (dash_pos != std::string::npos) {
        int start = std::stoi(item.substr(0, dash_pos));
        int end = std::stoi(item.substr(dash_pos + 1));
        if (start <= end) {
          for (int i = start; i <= end; ++i) {
            indices.push_back(i);
          }
        }
      } else {
        // 单个数字
        indices.push_back(std::stoi(item));
      }
    } catch (const std::exception&) {
      // 忽略无效的数字
      continue;
    }
  }

  // 去重并排序
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

  return indices;
}

bool Cli::FileExists(const std::string& file_path) {
  struct stat st;
  return (stat(file_path.c_str(), &st) == 0) && S_ISREG(st.st_mode);
}

bool Cli::DirectoryExists(const std::string& dir_path) {
  struct stat st;
  return (stat(dir_path.c_str(), &st) == 0) && S_ISDIR(st.st_mode);
}

bool Cli::CreateDirectory(const std::string& dir_path) {
  if (DirectoryExists(dir_path)) {
    return true;
  }

#ifdef _WIN32
  return _mkdir(dir_path.c_str()) == 0;
#else
  return mkdir(dir_path.c_str(), 0755) == 0;
#endif
}

std::string Cli::GetFileExtension(const std::string& file_path) {
  size_t pos = file_path.find_last_of('.');
  if (pos != std::string::npos) {
    return file_path.substr(pos + 1);
  }
  return "";
}

std::string Cli::GetBaseName(const std::string& file_path) {
  size_t slash_pos = file_path.find_last_of("/\\");
  size_t dot_pos = file_path.find_last_of('.');

  size_t start = (slash_pos != std::string::npos) ? slash_pos + 1 : 0;
  size_t end = (dot_pos != std::string::npos && dot_pos > start) ? dot_pos : file_path.length();

  return file_path.substr(start, end - start);
}

void Cli::SetErrorCallback(std::function<void(const std::string&)> callback) { error_callback_ = callback; }

std::string Cli::GetLastError() const { return last_error_; }

// 私有方法实现

void Cli::RegisterOptions() {
  // 这里可以注册更多的参数规格
  // 当前版本使用直接解析方式
}

int Cli::ParseArgument(const std::string& arg, const std::string& next_arg, CliOptions& options) {
  if (arg.empty()) return 0;

  if (arg[0] == '-') {
    if (arg.length() > 1 && arg[1] == '-') {
      return ParseLongArgument(arg, next_arg, options);
    } else {
      return ParseShortArgument(arg, next_arg, options);
    }
  } else {
    // 位置参数（目前不支持）
    return 0;
  }
}

int Cli::ParseLongArgument(const std::string& arg, const std::string& next_arg, CliOptions& options) {
  std::string param = arg.substr(2);  // 去掉"--"

  if (param == "help") {
    options.help = true;
    return 1;
  } else if (param == "version") {
    options.version = true;
    return 1;
  } else if (param == "single-scene") {
    options.mode = CliOptions::Mode::kSingleScene;
    return 1;
  } else if (param == "multi-scene") {
    options.mode = CliOptions::Mode::kMultiScene;
    return 1;
  } else if (param == "process-only") {
    options.mode = CliOptions::Mode::kProcessOnly;
    return 1;
  } else if (param == "batch") {
    options.mode = CliOptions::Mode::kBatchProcess;
    return 1;
  } else if (param == "interactive") {
    options.mode = CliOptions::Mode::kInteractive;
    return 1;
  } else if (param == "config") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--config requires a file path");
      return 0;
    }
    options.config_file = next_arg;
    return 2;
  } else if (param == "image") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--image requires a file path");
      return 0;
    }
    options.image_path = next_arg;
    return 2;
  } else if (param == "pointcloud") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--pointcloud requires a file path");
      return 0;
    }
    options.pointcloud_path = next_arg;
    return 2;
  } else if (param == "data-dir") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--data-dir requires a directory path");
      return 0;
    }
    options.data_directory = next_arg;
    return 2;
  } else if (param == "record-file") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--record-file requires a file path");
      return 0;
    }
    options.record_file = next_arg;
    return 2;
  } else if (param == "prefix") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--prefix requires a string");
      return 0;
    }
    options.output_prefix = next_arg;
    return 2;
  } else if (param == "format") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--format requires format specification");
      return 0;
    }
    std::stringstream ss(next_arg);
    std::string format;
    while (std::getline(ss, format, ',')) {
      options.output_formats.push_back(format);
    }
    return 2;
  } else if (param == "scenes") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--scenes requires scene selection");
      return 0;
    }
    options.scene_selection = next_arg;
    options.selected_scenes = ParseSceneSelection(next_arg);
    return 2;
  } else if (param == "max-scenes") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--max-scenes requires a number");
      return 0;
    }
    try {
      options.max_scenes = std::stoi(next_arg);
    } catch (...) {
      ReportError("Invalid number for --max-scenes: " + next_arg);
      return 0;
    }
    return 2;
  } else if (param == "rmse-threshold") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--rmse-threshold requires a number");
      return 0;
    }
    try {
      options.rmse_threshold = std::stod(next_arg);
    } catch (...) {
      ReportError("Invalid number for --rmse-threshold: " + next_arg);
      return 0;
    }
    return 2;
  } else if (param == "verbose") {
    options.verbose = true;
    return 1;
  } else if (param == "quiet") {
    options.quiet = true;
    return 1;
  } else if (param == "dry-run") {
    options.dry_run = true;
    return 1;
  } else if (param == "undistort") {
    options.undistort_preprocessing = true;
    return 1;
  } else if (param == "distortion-model") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--distortion-model requires a model name");
      return 0;
    }
    options.distortion_model = next_arg;
    return 2;
  } else if (param == "alpha") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--alpha requires a number");
      return 0;
    }
    try {
      options.alpha = std::stod(next_arg);
    } catch (...) {
      ReportError("Invalid number for --alpha: " + next_arg);
      return 0;
    }
    return 2;
  } else if (param == "output-dataset") {
    if (next_arg.empty() || next_arg[0] == '-') {
      ReportError("--output-dataset requires a directory path");
      return 0;
    }
    options.output_dataset_dir = next_arg;
    return 2;
  } else if (param == "preserve-original") {
    options.preserve_original = true;
    return 1;
  } else {
    return 0;  // 未知参数
  }
}

int Cli::ParseShortArgument(const std::string& arg, const std::string& next_arg, CliOptions& options) {
  if (arg.length() < 2) return 0;

  char flag = arg[1];

  switch (flag) {
    case 'h':
      options.help = true;
      return 1;
    case 's':
      options.mode = CliOptions::Mode::kSingleScene;
      return 1;
    case 'm':
      options.mode = CliOptions::Mode::kMultiScene;
      return 1;
    case 'p':
      options.mode = CliOptions::Mode::kProcessOnly;
      return 1;
    case 'b':
      options.mode = CliOptions::Mode::kBatchProcess;
      return 1;
    case 'i':
      options.mode = CliOptions::Mode::kInteractive;
      return 1;
    case 'c':
      if (next_arg.empty() || next_arg[0] == '-') {
        ReportError("-c requires a file path");
        return 0;
      }
      options.config_file = next_arg;
      return 2;
    case 'u':
      options.undistort_preprocessing = true;
      return 1;
    default:
      return 0;  // 未知短参数
  }
}

bool Cli::IsValidOutputFormat(const std::string& format) const {
  return std::find(kSupportedOutputFormats.begin(), kSupportedOutputFormats.end(), format) !=
         kSupportedOutputFormats.end();
}

std::string Cli::NormalizePath(const std::string& path) const {
  // 简单的路径规范化
  std::string normalized = path;

  // 替换反斜杠为正斜杠
  std::replace(normalized.begin(), normalized.end(), '\\', '/');

  // 去除尾部的斜杠
  while (!normalized.empty() && normalized.back() == '/') {
    normalized.pop_back();
  }

  return normalized;
}

void Cli::ReportError(const std::string& message) {
  last_error_ = message;

  if (error_callback_) {
    error_callback_(message);
  } else {
    std::cerr << ColorText("Error: ", kColorRed) << message << std::endl;
  }
}

std::string Cli::ColorText(const std::string& text, const std::string& color) const {
  // 检查是否支持颜色输出
#ifdef _WIN32
  return text;  // Windows 下简化处理
#else
  if (isatty(STDOUT_FILENO)) {
    return color + text + kColorReset;
  } else {
    return text;
  }
#endif
}