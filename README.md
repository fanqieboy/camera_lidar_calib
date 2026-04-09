# LiDAR-Camera Extrinsic Calibration Tool

这是一个基于 ArUco 标记和圆孔拟合的高精度 LiDAR-Camera 外参标定工具。该工具旨在通过单场景或多场景数据，自动解算雷达与相机之间的 6-DOF 离线外参变换矩阵。

## 核心特性
- **高鲁棒特征提取**：相机端通过识别 ArUco 标记并推导中心点，雷达端采用点云聚类与几何形状约束算法自动拟合标定板圆心。
- **全自动解算**：支持一键运行，从特征检测到外参解算（变换矩阵 $T_{L->C}$）全流程自动化。
- **可视化验证**：自动生成检测结果图与中间拟合文件，方便用户直观评估标定效果。
- **双模式运行**：支持实测数据标定，同时也内置了配套的仿真数据生成工具。
- **极简配置**：利用 YAML-cpp 管理参数，配置文件路径默认为 `../config/calib_config.yaml`。

## 环境依赖
- **OpenCV** (>= 4.0)
- **PCL** (Point Cloud Library)
- **Eigen 3**
- **YAML-cpp**
- **Ceres Solver**

## 快速上手

### 1. 编译
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. 生成仿真数据 (可选)
如果您暂时没有实测数据，可以使用内置脚本生成测试用的仿真数据：
```bash
cd scripts
python3 generate_test_data.py
```
该脚本将生成模拟的图像和点云文件供测试使用。

### 3. 执行前准备
在运行标定程序之前，请确保：
1. **相机内参文件**：在 `config/` 目录下已准备好相机的内参 YAML 文件（例如 `camera_intrinsics.yaml`）。
2. **正确配置路径**：在 `config/calib_config.yaml` 中配置好该内参文件的完整路径、待标定的图像路径及点云 PCD 路径。

### 4. 运行标定
```bash
cd build
./camera_lidar_calib
```

## 路径说明
- `config/`: 存放 `calib_config.yaml` 主配置文件及相机内参 YAML。
- `data/`: 存放输入的待标定数据（图像及 PCD 点云）。
- `include/`: 存放项目头文件。
- `src/`: 存放核心算法及主程序源代码。
- `scripts/`: 存放数据仿真与处理脚本。
- `output/`: 存放标定结果（变换矩阵、检测可视化、误差记录等）。
- `thirdparty/`: 项目依赖的第三方库（如 camera_model）。
