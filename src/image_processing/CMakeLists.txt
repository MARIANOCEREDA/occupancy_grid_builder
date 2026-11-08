cmake_minimum_required(VERSION 3.8)
project(image_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

set(COMPONENT_NAME ${PROJECT_NAME}_component)
set(NODE_NAME ${PROJECT_NAME}_node)

set(SOURCES src/image_undistort.cpp)
set(HEADERS include/image_processing/image_undistort.hpp)

add_library(${COMPONENT_NAME} SHARED ${SOURCES} ${HEADERS})
ament_target_dependencies(
  ${COMPONENT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  OpenCV
  cv_bridge
  sensor_msgs
)
rclcpp_components_register_node(${COMPONENT_NAME} PLUGIN "image_undistort::ImageUndistort" EXECUTABLE ${NODE_NAME})

target_include_directories(${COMPONENT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${COMPONENT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_package()
