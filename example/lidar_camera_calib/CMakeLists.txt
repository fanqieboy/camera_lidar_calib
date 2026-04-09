# LiDAR-Camera-Calib 独立构建 CMake 配置
cmake_minimum_required(VERSION 3.16)
project(lidar_camera_calib_standalone
    VERSION 2.0.0
    DESCRIPTION "独立运行的 LiDAR-相机标定工具"
    LANGUAGES CXX C
)

# 现代化 CMake 设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 构建类型配置
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

# 使用生成器表达式的现代编译标志
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG -Wall -Wno-sign-compare")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -DPCL_NO_PRECOMPILE -Wall -Wno-sign-compare")
endif ()

# 可视化功能开关选项
option(ENABLE_VISUALIZATION "Enable PCL visualization support" ON)
option(BUILD_TESTS "Build unit tests" OFF)
option(BUILD_EXAMPLES "Build example programs" OFF)

# 使用现代 CMake 查找必需包
find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
    features
    filters
    segmentation
    sample_consensus
    registration
    visualization
)

find_package(OpenCV 4.0 REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    calib3d
    aruco
    highgui
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 查找 yaml-cpp，使用回退策略
find_package(yaml-cpp QUIET)
if (yaml-cpp_FOUND AND TARGET yaml-cpp)
  # 找到现代 cmake 目标，创建别名
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
else ()
  # 尝试使用 pkg-config 方式
  find_package(PkgConfig QUIET)
  if (PKG_CONFIG_FOUND)
    pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
    # 为保持一致性创建导入目标
    if (NOT TARGET yaml-cpp::yaml-cpp)
      add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
      set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIRS}"
          INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIBRARIES}"
          INTERFACE_LINK_DIRECTORIES "${YAML_CPP_LIBRARY_DIRS}"
      )
    endif ()
  else ()
    # 最后手段：假设系统安装
    message(WARNING "通过 cmake 或 pkg-config 未找到 yaml-cpp，使用系统安装版本")
    add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
    set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
        INTERFACE_LINK_LIBRARIES "yaml-cpp"
    )
  endif ()
endif ()

# 查找 Boost（某些 PCL 组件需要）
find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)

# 检查 std::filesystem 是否可用，或者需要链接 stdc++fs
include(CheckIncludeFileCXX)
check_include_file_cxx("filesystem" HAVE_STD_FILESYSTEM)
if (NOT HAVE_STD_FILESYSTEM)
  # 检查实验性 filesystem
  check_include_file_cxx("experimental/filesystem" HAVE_STD_EXPERIMENTAL_FILESYSTEM)
  if (HAVE_STD_EXPERIMENTAL_FILESYSTEM)
    set(FILESYSTEM_LIB "stdc++fs")
  else ()
    message(WARNING "未找到 <filesystem> 或 <experimental/filesystem>")
  endif ()
endif ()

# 按模块组织的源文件
set(LIDAR_CAMERA_CALIB_CORE_SOURCES
    src/common_lib.cpp
    src/config_manager.cpp
    src/data_loader.cpp
    src/calib_core.cpp
    src/result_exporter.cpp
)

set(LIDAR_CAMERA_CALIB_DETECTOR_SOURCES
    src/qr_detector.cpp
    src/lidar_detector.cpp
)

set(LIDAR_CAMERA_CALIB_CLI_SOURCES
    src/cli.cpp
)

# 合并所有源文件
set(LIDAR_CAMERA_CALIB_SOURCES
    ${LIDAR_CAMERA_CALIB_CORE_SOURCES}
    ${LIDAR_CAMERA_CALIB_DETECTOR_SOURCES}
    ${LIDAR_CAMERA_CALIB_CLI_SOURCES}
)

# 根据条件添加可视化器
if (ENABLE_VISUALIZATION)
  list(APPEND LIDAR_CAMERA_CALIB_SOURCES src/visualizer.cpp)
endif ()

# ----- 设置生成thirdpart .so的路径 -----
set(SHARED_LIB_PATH ${CMAKE_BINARY_DIR}/lib)
message(STATUS "SHARED_LIB_PATH: ${SHARED_LIB_PATH}")

# 相机模型
add_subdirectory(thirdparty/camera_model)

# 添加新的库子目录
add_subdirectory(src/dataset_loader)
add_subdirectory(src/pcd_merger)
add_subdirectory(src/video_extractor)

add_library(lidar_camera_calib_lib SHARED
    ${LIDAR_CAMERA_CALIB_SOURCES}
)

# 设置目标属性
set_target_properties(lidar_camera_calib_lib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# 目标特定的包含目录
target_include_directories(lidar_camera_calib_lib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 目标特定的编译定义
target_compile_definitions(lidar_camera_calib_lib
    PUBLIC
    LIDAR_CAMERA_CALIB_STANDALONE
    $<$<BOOL:${ENABLE_VISUALIZATION}>:ENABLE_VISUALIZATION>
    PRIVATE
    ${PCL_DEFINITIONS}
)

# 使用导入目标的现代目标链接
target_link_libraries(lidar_camera_calib_lib
    PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    dataset_loader
    pcd_merger
    video_extractor
    PRIVATE
    yaml-cpp::yaml-cpp
    Boost::system
    Boost::filesystem
    $<$<BOOL:${FILESYSTEM_LIB}>:${FILESYSTEM_LIB}>
)


# 主可执行文件
add_executable(lidar-camera-calib
    src/main_standalone.cpp
    src/exception_handler.cpp
    src/undistortion_preprocessor.cpp
)

# 将可执行文件链接到库
target_link_libraries(lidar-camera-calib
    PRIVATE
    camera_model
    lidar_camera_calib_lib
)
target_link_libraries(lidar-camera-calib PRIVATE -lstdc++fs)
# 设置可执行文件属性
set_target_properties(lidar-camera-calib PROPERTIES
    OUTPUT_NAME "lidar-camera-calib"
)

# --- 新增部分 ---
# 独立的反向畸变恢复可执行文件
add_executable(recover_distortion
    recover_distortion_main.cpp
    reverse_distortion_map.cc
)

target_link_libraries(recover_distortion
    PRIVATE
    camera_model
    lidar_camera_calib_lib
    yaml-cpp::yaml-cpp # Link yaml-cpp which should already be available from the main project setup
    -lstdc++fs # Link stdc++fs for std::filesystem
)

# 设置可执行文件属性
set_target_properties(recover_distortion PROPERTIES
    OUTPUT_NAME "recover_distortion"
)
# --- 新增部分结束 ---

# 如果启用则构建测试
if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif ()

# 如果启用则构建示例
if (BUILD_EXAMPLES)
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
    add_subdirectory(examples)
  else ()
    message(STATUS "示例构建已启用但未找到示例目录")
  endif ()
endif ()


# 增强的配置摘要
message(STATUS "")
message(STATUS "============================================")
message(STATUS "  LiDAR-Camera-Calib Standalone Configuration")
message(STATUS "============================================")
message(STATUS "Project Info:")
message(STATUS "  Name: ${PROJECT_NAME}")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Description: ${PROJECT_DESCRIPTION}")
message(STATUS "")
message(STATUS "Build Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  PCL: ${PCL_VERSION}")
message(STATUS "  OpenCV: ${OpenCV_VERSION}")
message(STATUS "  Eigen3: ${Eigen3_VERSION}")
if (TARGET yaml-cpp::yaml-cpp)
  message(STATUS "  yaml-cpp: Found (target)")
else ()
  message(STATUS "  yaml-cpp: Found (legacy)")
endif ()
if (Boost_FOUND)
  message(STATUS "  Boost: ${Boost_VERSION} (${Boost_LIBRARIES})")
else ()
  message(STATUS "  Boost: Not found")
endif ()
if (FILESYSTEM_LIB)
  message(STATUS "  Filesystem: ${FILESYSTEM_LIB}")
else ()
  message(STATUS "  Filesystem: std::filesystem")
endif ()
message(STATUS "")
message(STATUS "Build Options:")
message(STATUS "  Visualization: ${ENABLE_VISUALIZATION}")
message(STATUS "  Build Tests: ${BUILD_TESTS}")
message(STATUS "  Build Examples: ${BUILD_EXAMPLES}")
message(STATUS "  Export Compile Commands: ${CMAKE_EXPORT_COMPILE_COMMANDS}")
message(STATUS "")
message(STATUS "Targets:")
message(STATUS "  Library: lidar_camera_calib_lib (LidarCameraCalib::lidar_camera_calib_lib)")
message(STATUS "  Executable: lidar-camera-calib")
message(STATUS "============================================")
message(STATUS "")

# 如果启用了测试，添加测试可执行文件
if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif ()

