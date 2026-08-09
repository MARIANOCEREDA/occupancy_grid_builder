cmake_minimum_required(VERSION 3.8)
project(image_undistort)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

set(COMPONENT_NAME ${PROJECT_NAME}_component)
set(NODE_NAME ${PROJECT_NAME}_node)

set(SOURCES src/image_undistort.cpp)
set(HEADERS include/image_undistort/image_undistort.hpp)

add_executable(${NODE_NAME} ${SOURCES} ${HEADERS})
ament_target_dependencies(
  ${NODE_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  OpenCV
  image_transport
  cv_bridge
  sensor_msgs
)

add_library(${COMPONENT_NAME} SHARED ${SOURCES} ${HEADERS})
ament_target_dependencies(
  ${COMPONENT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  OpenCV
  image_transport
  cv_bridge
  sensor_msgs
)
rclcpp_components_register_nodes(${COMPONENT_NAME} "image_undistort::ImageUndistort")

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${COMPONENT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${COMPONENT_NAME}
  LIBRARY DESTINATION lib
)

install(
  TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
