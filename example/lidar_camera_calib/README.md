# LiDAR-Camera-Calib

LiDAR-相机外参标定工具的独立版本。提供单场景与多场景标定流程、点云与图像的目标检测、可视化与结果导出能力，
适用于自动驾驶/机器人等多传感器融合场景。

- 语言/标准：C++17
- 可执行程序：`lidar-camera-calib`
- 版本：v2.0.0+（见 `CMakeLists.txt`）
- **重要更新**：v2.0.0 移除了实时 PCL 可视化，改为文件保存方式，提升服务器兼容性；v2.2+ 新增了去畸变预处理功能

## 核心特性

- 单场景与多场景标定：支持目录批量与记录文件两种多场景输入
- QR/圆形目标检测：OpenCV ArUco + PCL 圆拟合与平面/边界提取
- 结果导出：TXT/YAML/JSON/XML/CSV，多格式可选，支持着色点云
- 结果可视化：自动保存点云为 PCD 文件，支持图像显示和中间结果导出
- 详细 CLI：便捷的命令行参数与示例，内置帮助/版本显示
- **新特性**：支持多种畸变模型的图像/视频去畸变预处理（v2.2+）
- **新特性**：支持多朝向相机-雷达外参标定（camera_direction参数，v2.3+）
- **新特性**：自动输出标定结果的详细解读信息，包括变换矩阵、位置和姿态（v2.3+）

## 目录结构

- `src/`：核心实现
  - 核心模块：`calib_core`、`qr_detector`、`lidar_detector`、`result_exporter`、`cli`、`visualizer`
  - 数据处理：`dataset_loader`、`pcd_merger`、`video_extractor` (v2.1新增)
  - 预处理：`undistortion_preprocessor` (v2.2+新增)
  - 工具模块：`config_manager`、`data_loader`、`exception_handler`、`common_lib`、`logger`
- `include/`：公开头文件（与 `src/` 一一对应）
- `thirdparty/`：第三方库（如 `camera_model`，用于支持多种相机模型）
- `tests/`：单元、集成（`tests/integration/`）与性能（`tests/performance/`）测试
- `config/`：YAML 配置模板与参数说明
- `examples/`：示例程序与用法参考
- `docs/`：安装与配置等文档
- `build/`：本地构建输出（应忽略）
- `pics/`：图片资源

## 依赖

- 必需：PCL(>=1.8)、OpenCV(>=4.0，含 `aruco`)、Eigen3(>=3.3)、yaml-cpp
- 可选：Boost（部分 PCL 组件）

## 快速构建

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DENABLE_VISUALIZATION=ON
make -j$(nproc)
```

- 运行：`./lidar-camera-calib --help`
- 如需 Debug/覆盖率/自定义安装前缀等，请参考 `docs/installation.md`
- 构建测试（可选）：`cmake .. -DBUILD_TESTS=ON && make && make run_tests`
- 构建示例（可选）：`cmake .. -DBUILD_EXAMPLES=ON && make`

## 快速开始

- 最简单单场景（使用默认配置 `config/calib_config_standalone.yaml`）：

```bash
./lidar-camera-calib --image path/to/camera.png --pointcloud path/to/lidar.pcd
```

程序执行流程说明文档：`docs/program_execution_flow.md`

- 指定配置并启用：

```bash
./lidar-camera-calib -c config/calib_config_standalone.yaml
```

- 从目录批量取第一对数据进行单场景：

```bash
./lidar-camera-calib --data-dir calib_data/
```

- 多场景（从目录收集，导出 YAML 和 JSON）：

```bash
./lidar-camera-calib --multi-scene --data-dir calib_data/ --format yaml,json
```

- 多场景（从记录文件选择部分场景）：

```bash
./lidar-camera-calib -m --record-file centers.txt --scenes "1,3,5-8"
```

- 仅处理（不做标定，便于调参数/观测中间结果）：

```bash
./lidar-camera-calib --process-only --data-dir test_data
```

### 命令行用法（摘录） 运行 `./lidar-camera-calib --help` 查看完整清单

- 模式：`--single-scene|-s`、`--multi-scene|-m`、`--process-only|-p`、`--batch|-b`、`--interactive|-i`
- 输入：`--config|-c FILE`、`--image IMAGE`、`--pointcloud PC`、`--data-dir DIR`、`--record-file FILE`
- 输出：`--prefix PREFIX`、`--format txt,yaml,json,xml,csv`
- 处理：`--scenes "1,3,5-8,10"`、`--max-scenes N`、`--rmse-threshold TH`
- 通用：`--verbose`、`--quiet`、`--dry-run`、`--help|-h`、`--version`

## 配置与数据约定

- 配置模板：`config/calib_config_standalone.yaml` 与 `config/calib_config_template.yaml`
- 配置详解与传感器示例：`docs/configuration.md`
- 数据输入：
    - 单场景可直接指定 `--image` 与 `--pointcloud`
    - 仅指定 `--image` 时，程序按约定尝试匹配同名 `.pcd`
    - 目录输入 `--data-dir` 会自动成对匹配常见命名的图像与点云
    - **新支持**：特定数据集目录结构（见下方数据集格式）
- 输出设置：
    - **输出目录**：只能通过配置文件中的 `output_path` 参数设置（默认 `./output`），**不再支持命令行参数**
    - **输出前缀**：可通过 `--prefix` 命令行参数自定义输出文件前缀
    - 点云结果保存和中间结果导出通过配置文件设置

## 新增功能：去畸变预处理（v2.2+）

LiDAR-Camera-Calib 现在支持鱼眼和普通镜头的图像/视频去畸变预处理，兼容 Kalibr 完整相机模型集合：
- 相机投影模型: pinhole, omni, ds, eucm
- 畸变模型: radtan, equi, fov, none
- 支持所有有效的模型组合

## 新增功能：多朝向相机-雷达标定（v2.3+）

LiDAR-Camera-Calib 现在支持多朝向相机-雷达标定，通过camera_direction参数指定相机与雷达的相对朝向：
- 0: 正前方 (基准，坐标不变)
- 1: 相机在雷达背后 (绕Z轴旋转180度)
- 2: 相机在雷达左侧 (绕Z轴旋转-90度)
- 3: 相机在雷达右侧 (绕Z轴旋转+90度)

该功能允许用户处理不同安装方向的相机-雷达系统。


### 相机方向配置示例
在 `config/calib_config_standalone.yaml` 中新增相机方向配置：
```yaml
# 相机方向参数
camera_direction: 0    # 相机朝向 (0:正前方,1:背后,2:左侧,3:右侧)
```

### 使用流程
1. 首先通过其他工具（如 camera_calib、Kalibr）计算相机内参和畸变参数，并保存为 YAML 文件
2. 在 LiDAR-Camera-Calib 配置文件中设置去畸变参数和相机方向参数
3. 运行标定，系统将自动进行去畸变预处理和朝向校正

## 支持的数据集格式

### 特定数据集格式（v2.1 新增）

LiDAR-Camera-Calib 现在支持特定的目录结构，专为多PCD文件合并和视频帧提取设计：

```
dataset_root/
├── cameras/
│   ├── camera_0/
│   │   ├── video.mp4       # 主视频文件
│   │   ├── video2.mp4      # 可选的额外视频
│   │   └── labels.txt      # 标注文件（可选，将被忽略）
│   ├── camera_1/
│   │   └── video.mp4
│   └── camera_N/
│       └── ...
└── lidar/
    ├── 001.pcd            # PCD文件按文件名排序
    ├── 002.pcd
    ├── 003.pcd
    └── ...
```

**使用新数据集格式：**

新数据集的参数需要通过配置文件设置。请修改 `config/calib_config_standalone.yaml` 中的以下参数：

```yaml
# 新数据集支持参数 (Feature 001-1-pcd-pcd)
dataset_dir: "/path/to/dataset_root"     # 数据集根目录 (包含 cameras/ 和 lidar/ 目录)
camera_id: "camera_0"                    # 相机ID过滤 (如 "camera_0")，空字符串表示所有相机
max_pcds: 50                             # 最大PCD文件合并数量，-1表示不限制
```

然后运行标定：

```bash
# 使用配置文件中的数据集设置
./lidar-camera-calib -c config/calib_config_standalone.yaml

# 启用点云保存和中间结果导出需通过配置文件设置
./lidar-camera-calib -c config/calib_config_standalone.yaml
```

**配置参数说明：**

- `dataset_dir`：数据集根目录，必须包含 `cameras/` 和 `lidar/` 子目录
- `camera_id`：指定使用的相机（如 "camera_0"），空字符串表示使用所有可用相机
- `max_pcds`：限制合并的PCD文件数量，-1表示不限制，按文件名顺序选择
- `output_path`：输出目录路径，默认为 `./output`（**仅可通过配置文件设置，不支持命令行参数**）
- `enable_visualization`：是否启用点云和图像结果保存，仅可通过配置文件设置
- `save_intermediate_results`：是否保存中间处理结果，仅可通过配置文件设置

**特性说明：**

1. **多PCD文件合并**：自动按文件名排序合并多个PCD文件为统一点云
2. **视频帧提取**：从MP4视频文件中提取帧作为标定图像
3. **内存优化**：支持大数据集的流式处理和内存限制
4. **静止场景**：适用于设备静止的标定场景，无需时间同步
5. **灵活过滤**：可指定相机ID和PCD文件数量限制

### 传统数据格式

继续支持传统的图像+点云对格式：

```
data_dir/
├── scene1.png & scene1.pcd
├── scene2.jpg & scene2.pcd
└── ...
```

## 结果输出与可视化

LiDAR-Camera-Calib 会自动将处理结果保存为文件，无需实时可视化界面，适合服务器环境和批处理场景。

### 输出文件结构

所有结果文件保存在配置的输出目录中（默认 `./output/`）：

```
output/
├── pointclouds/                    # 点云文件 (.pcd)
│   ├── original_cloud.pcd          # 原始点云（着色）
│   ├── filtered_cloud.pcd          # 滤波后点云
│   ├── plane_cloud.pcd             # 平面点云
│   ├── aligned_cloud.pcd           # 对齐后点云
│   ├── edge_cloud.pcd              # 边界点云
│   ├── final_centers.pcd           # 最终检测的圆心
│   ├── qr_centers.pcd              # QR检测的圆心
│   ├── calibrated_cloud.pcd        # 标定后的着色点云
│   └── failed_*.pcd                # 失败诊断文件（如有）
├── images/                         # 图像文件 (.png/.jpg)
│   ├── camera_image.png            # 原始相机图像
│   ├── undistorted_image.png       # 去畸变后的图像（如果启用去畸变功能）
│   ├── qr_detection_result.png     # QR检测结果图像
│   └── failed_*.png                # 失败诊断图像（如有）
├── calib_result.txt                # 标定结果（文本格式）
├── calib_result.yaml               # 标定结果（YAML格式）
├── calib_result.json               # 标定结果（JSON格式）
└── ...                            # 其他格式结果文件
```

### 查看结果

**点云可视化：**
- 使用 [CloudCompare](https://www.cloudcompare.org/) 查看 PCD 文件
- 使用 PCL Viewer：`pcl_viewer original_cloud.pcd`
- 使用 Python：`pip install open3d` 然后加载 PCD 文件

**图像查看：**
- 任何图像查看器都可以打开 PNG/JPG 文件
- OpenCV 在处理过程中会显示关键图像（如 QR 检测结果）

**标定结果：**
- 直接查看 TXT/YAML/JSON 文件获得变换矩阵和精度信息
- 导入到其他程序中使用标定参数

**相机方向结果：**
- 当使用camera_direction参数时，输出的变换矩阵已自动应用了朝向校正
- 在输出的文本信息中可以看到经过朝向校正后的变换矩阵和欧拉角



### 配置输出选项

在 `config/calib_config_standalone.yaml` 中配置：

```yaml
# 输出控制
output_path: "./output"              # 输出目录
enable_visualization: true          # 是否保存点云和图像文件
save_intermediate_results: true     # 是否保存中间处理结果

# 相机方向参数
camera_direction: 0                 # 相机朝向 (0:正前方,1:背后,2:左侧,3:右侧)

# 输出控制
enable_visualization: true          # 是否启用可视化
save_intermediate_results: true     # 是否保存中间处理结果
```


### 相机方向参数的使用说明

camera_direction参数允许用户处理不同安装方向的相机-雷达系统：
- 设置camera_direction为0表示相机与雷达同向（正前方）
- 设置为1表示相机与雷达反向（背后）
- 设置为2表示相机相对于雷达向左（左侧）
- 设置为3表示相机相对于雷达向右（右侧）

标定结果会自动应用相应的坐标变换，以得到正确的相机-雷达外参。

### 外参结果输出

标定完成后，系统会自动输出详细的结果解读信息：
- 变换矩阵：显示完整的4x4变换矩阵
- 位置信息：以米为单位显示相机在雷达坐标系中的X(前)、Y(左)、Z(上)位置
- 姿态信息：以ZYX欧拉角格式显示相机的roll(横滚)、pitch(俯仰)、yaw(偏航)角度
- 校正后结果：当使用camera_direction参数时，还会显示经过朝向校正后的结果


