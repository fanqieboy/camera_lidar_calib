# LiDAR-Camera Extrinsic Calibration Tool

这是一个基于 ArUco 标记和圆孔拟合的高精度 LiDAR-Camera 外参标定工具。该工具旨在通过单场景（One-shot）或多场景数据，自动解算雷达与相机之间的 6-DOF 离线外参变换矩阵。

## 核心特性
- **相机端 ArUco 检测**：通过识别 ArUco 标记并推导中心点，确保在复杂光下具有极高的魯棒性。
- **雷达端圆孔拟合**：采用高性能点云聚类与几何形状约束算法，在原始数据中自动识别、拟合物理标定板。
- **全自动解算流水线**：从加载配置、传感器特征检测、特征点对照到外参解算（变换矩阵 $T_{L->C}$），全自动完成，通常耗时不足 1 秒。
- **可视化结果**：自动生成相机端检测效果图与雷达端中间拟合文件，便于精度评估。
- **极简配置**：利用 YAML-cpp 管理配置参数，支持命令行动态指定配置文件。

## 环境依赖
- **OpenCV** (>= 4.0)
- **PCL** (Point Cloud Library)
- **Eigen 3**
- **YAML-cpp**
- **Ceres Solver** (可选，取决于解算核心实现)

## 目录结构
- `src/`: 核心标定逻辑、检测器实现。
- `include/`: API 头文件。
- `config/`: 存放 `calib_config.yaml` 配置文件。
- `data/`: 存放输入数据（图像、PCD 点云文件）。
- `output/`: 存放标定结果（变换矩阵、检测可视化、误差记录）。
- `example/`: 示例数据集。

## 快速上手

### 1. 编译
```bash
# 进入 camera_lidar_calib 目录
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. 准备数据
将待标定的图像和 .pcd 文件放入 `data/` 目录，并确保在 `config/calib_config.yaml` 中正确配置了对应的路径及内参。

### 3. 运行标定
```bash
cd build
./camera_lidar_calib ../config/calib_config.yaml
```

## 注意事项
- 请确保相机内参（及畸变模型）在配置文件中已预先准确设定。
- 标定板特征（如圆周半径等）对拟合精度至关重要，请测量后填入配置。
