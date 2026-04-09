# Dataset Loader Library

set(DATASET_LOADER_SOURCES
    dataset_loader.cpp
    dataset_structure.cpp
)

add_library(dataset_loader ${DATASET_LOADER_SOURCES})

target_include_directories(dataset_loader
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include/dataset_loader
    PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(dataset_loader
    PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
)

# Set C++17 standard
target_compile_features(dataset_loader PUBLIC cxx_std_17)

# Compiler-specific options
target_compile_definitions(dataset_loader PRIVATE ${PCL_DEFINITIONS})