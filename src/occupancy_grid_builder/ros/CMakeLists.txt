cmake_minimum_required(VERSION 3.8)
project(occupancy_grid_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

# Find the occupancy_grid_builder_cpp C++ library
if(DEFINED OCCUPANCY_GRID_BUILDER_CPP)
  # Using the library built in the cpp/ folder
  set(occupancy_grid_builder_cpp_DIR "${OCCUPANCY_GRID_BUILDER_CPP}/install/lib/cmake/occupancy_grid_builder_cpp")
  set(occupancy_grid_builder_cpp_INCLUDE_DIR "${OCCUPANCY_GRID_BUILDER_CPP}/install/include")
  set(occupancy_grid_builder_cpp_LIB_DIR "${OCCUPANCY_GRID_BUILDER_CPP}/install/lib")
  find_package(occupancy_grid_builder_cpp REQUIRED PATHS ${occupancy_grid_builder_cpp_DIR} NO_DEFAULT_PATH)
else()
  find_package(occupancy_grid_builder_cpp REQUIRED)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${occupancy_grid_builder_cpp_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS} ${occupancy_grid_builder_cpp_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})

set(COMPONENT_NAME ${PROJECT_NAME}_component)
set(NODE_NAME ${PROJECT_NAME}_node)

set(SOURCES src/occupancy_grid_builder_node.cpp)
set(HEADERS include/occupancy_grid_builder/occupancy_grid_builder_node.hpp)

add_library(${COMPONENT_NAME} SHARED ${SOURCES} ${HEADERS})
ament_target_dependencies(
  ${COMPONENT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  lifecycle_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(${COMPONENT_NAME} occupancy_grid_builder_cpp ${PCL_LIBRARIES})
rclcpp_components_register_node(${COMPONENT_NAME} PLUGIN "occupancy_grid_builder::OccupancyGridBuilderNode" EXECUTABLE ${NODE_NAME})

target_include_directories(${COMPONENT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)

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
