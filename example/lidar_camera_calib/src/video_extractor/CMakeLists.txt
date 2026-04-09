# Video Extractor Library

set(VIDEO_EXTRACTOR_SOURCES
    video_extractor.cpp
    video_types.cpp
)

add_library(video_extractor ${VIDEO_EXTRACTOR_SOURCES})

target_include_directories(video_extractor
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include/video_extractor
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(video_extractor
    PUBLIC
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
)

# Set C++17 standard
target_compile_features(video_extractor PUBLIC cxx_std_17)