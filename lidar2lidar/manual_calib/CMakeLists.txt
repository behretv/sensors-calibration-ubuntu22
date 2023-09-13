cmake_minimum_required(VERSION 2.8.3)
project(LidarToLidar)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
set(CMAKE_CXX_STANDARD 14)

# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Boost
find_package(Boost REQUIRED system)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories(${EIGEN_ROOT})

# Jsoncpp
find_package(jsoncpp REQUIRED)

# Link local libraries
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ${JSONCPP_LIBRARIES} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

add_executable(run_lidar2lidar src/run_lidar2lidar.cpp)
target_link_libraries(run_lidar2lidar ${PROJECT_NAME})
target_link_libraries(run_lidar2lidar ${PCL_LIBRARIES})
target_link_libraries(run_lidar2lidar ${Pangolin_LIBRARIES})
