project(camera_model)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -fopenmp -Wno-all")

find_package(Boost REQUIRED COMPONENTS
    filesystem
    program_options
    system)
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")


# OpenCV
find_package(OpenCV 4.0 QUIET
    COMPONENTS
    core
    imgproc
    calib3d
    highgui
)
if (NOT OpenCV_FOUND)
  find_package(OpenCV 4.0 PATHS ${OpenCV_LIB_DIR} NO_DEFAULT_PATH REQUIRED
      COMPONENTS
      core
      imgproc
      calib3d
      highgui
  )
endif ()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

add_library(${PROJECT_NAME} SHARED
    ./src/camera_models/Camera.cc
    ./src/camera_models/CameraFactory.cc
    ./src/camera_models/PinholeCamera.cc
    ./src/camera_models/PinholeFullCamera.cc
    ./src/camera_models/CataCamera.cc
    ./src/camera_models/EquidistantCamera.cc
    ./src/camera_models/ScaramuzzaCamera.cc
    ./src/camera_models/PolyFisheyeCamera.cpp
    ./src/camera_models/SplineCamera.cpp
    ./src/camera_models/FovCamera.cpp
    ./src/code_utils/math_utils/Polynomial.cpp
    ./src/gpl/gpl.cc
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${SHARED_LIB_PATH}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)


