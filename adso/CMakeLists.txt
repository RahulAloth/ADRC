cmake_minimum_required(VERSION 3.10)
project(adso LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
    
file(GLOB SOURCES
    src/*.cpp
)

add_executable(adso ${SOURCES})

target_link_libraries(adso
    ${OpenCV_LIBS}
)
