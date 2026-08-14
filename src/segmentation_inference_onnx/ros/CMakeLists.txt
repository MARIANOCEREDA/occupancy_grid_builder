cmake_minimum_required(VERSION 3.8)
project(segmentation_inference_onnx_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

set(YOLO_CPP_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../cpp/build/install"
  CACHE PATH "Installation prefix of the yolo_onnx_inference C++ library")

if(NOT EXISTS "${YOLO_CPP_PREFIX}/lib/libyolo_onnx_inference.so")
  message(FATAL_ERROR
  "libyolo_onnx_inference.so not found in '${YOLO_CPP_PREFIX}/lib'.\n"
  "Run 'make build-segmentation-inference-onnx-cpp' from the workspace root first,\n"
  "or set -DYOLO_CPP_PREFIX=<path> to the library installation prefix.")
endif()

list(PREPEND CMAKE_PREFIX_PATH "${YOLO_CPP_PREFIX}")
find_package(yolo_onnx_inference CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/inference_node.cpp
  include/segmentation_inference_onnx/inference_node.hpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  vision_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  yolo_onnx_inference::yolo_onnx_inference
  ${OpenCV_LIBS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yolo_onnx_inference_ros::InferenceNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    sensor_msgs
    vision_msgs
    cv_bridge
    geometry_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY
  include/
  DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install model files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../models")
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../models
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Copy the shared library next to the node so the loader finds it at runtime.
install(FILES "${YOLO_CPP_PREFIX}/lib/libyolo_onnx_inference.so"
    DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
