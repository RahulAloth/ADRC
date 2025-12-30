cmake_minimum_required(VERSION 3.10)
project(adso_nvmm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

include_directories(
    ${GST_INCLUDE_DIRS}
    ${GST_VIDEO_INCLUDE_DIRS}
)

link_directories(
    ${GST_LIBRARY_DIRS}
    ${GST_VIDEO_LIBRARY_DIRS}
)

add_executable(jetson_gst_pipeline
    src/jetson_gst_pipeline.cpp
)

target_link_libraries(jetson_gst_pipeline
    ${GST_LIBRARIES}
    ${GST_VIDEO_LIBRARIES}
)