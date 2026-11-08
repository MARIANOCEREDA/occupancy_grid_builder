cmake_minimum_required(VERSION 3.8)
project(pcl_labeler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

# Find the mask_lidar_fusion C++ library
if(DEFINED CAMERA_LIDAR_FUSION_CPP)
  # Using the library built in the cpp/ folder
  set(mask_lidar_fusion_DIR "${CAMERA_LIDAR_FUSION_CPP}/install/lib/cmake/mask_lidar_fusion")
  set(mask_lidar_fusion_INCLUDE_DIR "${CAMERA_LIDAR_FUSION_CPP}/install/include")
  set(mask_lidar_fusion_LIB_DIR "${CAMERA_LIDAR_FUSION_CPP}/install/lib")
  find_package(mask_lidar_fusion REQUIRED PATHS ${mask_lidar_fusion_DIR} NO_DEFAULT_PATH)
else()
  find_package(mask_lidar_fusion REQUIRED)
endif()

include_directories(include ${PCL_INCLUDE_DIRS} ${mask_lidar_fusion_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${mask_lidar_fusion_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})

set(COMPONENT_NAME ${PROJECT_NAME}_component)
set(NODE_NAME ${PROJECT_NAME}_node)

set(SOURCES src/pcl_labeler_node.cpp)
set(HEADERS include/pcl_labeler/pcl_labeler_node.hpp)

add_library(${COMPONENT_NAME} SHARED ${SOURCES} ${HEADERS})
ament_target_dependencies(
  ${COMPONENT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  OpenCV
  cv_bridge
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(${COMPONENT_NAME} mask_lidar_fusion ${PCL_LIBRARIES})
rclcpp_components_register_node(${COMPONENT_NAME} PLUGIN "pcl_labeler::PCLLabelerNode" EXECUTABLE ${NODE_NAME})

target_include_directories(${COMPONENT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${COMPONENT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
