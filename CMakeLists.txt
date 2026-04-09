cmake_minimum_required(VERSION 3.10)
project(lidar_camera_calib VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必需的系统库
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含路径设置
include_directories(
    ./include
    ./thirdparty/camera_model/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# 库目录设置
link_directories(
    ${PCL_LIBRARY_DIRS}
)

# 核心代码源文件（除去你自己后续开发的 main.cpp）
# 这里的源文件对应你通过 API 调用的现成库
set(CORE_SOURCES
    src/calib_core.cpp
    src/lidar_detector.cpp
    src/qr_detector.cpp
    src/config_manager.cpp
    src/data_loader.cpp
    src/logger.cpp
    src/result_exporter.cpp
    src/visualizer.cpp
    src/common_lib.cpp
    # 全量相机投影模型 (Fix vtable linkage)
    thirdparty/camera_model/src/camera_models/Camera.cc
    thirdparty/camera_model/src/camera_models/CameraFactory.cc
    thirdparty/camera_model/src/camera_models/PinholeCamera.cc
    thirdparty/camera_model/src/camera_models/PinholeFullCamera.cc
    thirdparty/camera_model/src/camera_models/EquidistantCamera.cc
    thirdparty/camera_model/src/camera_models/FovCamera.cpp
    thirdparty/camera_model/src/camera_models/ScaramuzzaCamera.cc
    thirdparty/camera_model/src/camera_models/CataCamera.cc
    thirdparty/camera_model/src/camera_models/PolyFisheyeCamera.cpp
    thirdparty/camera_model/src/camera_models/SplineCamera.cpp
)

# 编译主程序（你自己即将编写的 main.cpp）
# 这里生成一个名为 camera_lidar_calib 的可执行文件
add_executable(camera_lidar_calib src/main.cpp ${CORE_SOURCES}
    # 核心数学与几何支持
    thirdparty/camera_model/src/code_utils/math_utils/Polynomial.cpp
    thirdparty/camera_model/src/gpl/gpl.cc
)

# 链接库文件
target_link_libraries(camera_lidar_calib
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
    pthread
)

message(STATUS "### INFO: Lidar-Camera Calibration Project Configured Successfully ###")
