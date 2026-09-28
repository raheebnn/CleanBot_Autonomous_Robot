# Build template for slam_receiver against a local ORB-SLAM3 build.
#   mkdir -p build && cd build
#   cmake .. -DORB_SLAM3_DIR=$HOME/ORB_SLAM3
#   make -j4
cmake_minimum_required(VERSION 3.10)
project(cleanbot_slam_receiver CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(ORB_SLAM3_DIR "$ENV{HOME}/ORB_SLAM3" CACHE PATH "Path to the ORB-SLAM3 source/build directory")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

add_executable(slam_receiver slam_receiver.cc)

target_link_libraries(slam_receiver
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Threads::Threads
  -lboost_serialization
  -lcrypto
)
