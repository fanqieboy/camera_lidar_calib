# PCD Merger Library

set(PCD_MERGER_SOURCES
    pcd_merger.cpp
    merged_point_cloud.cpp
)

add_library(pcd_merger ${PCD_MERGER_SOURCES})

target_include_directories(pcd_merger
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include/pcd_merger
    PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(pcd_merger
    PUBLIC
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

# Set C++17 standard
target_compile_features(pcd_merger PUBLIC cxx_std_17)

# Compiler-specific options
target_compile_definitions(pcd_merger PRIVATE ${PCL_DEFINITIONS})