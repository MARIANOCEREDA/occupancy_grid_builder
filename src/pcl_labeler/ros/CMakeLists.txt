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
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(nlohmann_json REQUIRED)

# Find the pcl_labeler_cpp C++ library
if(DEFINED PCL_LABELER_CPP)
  # Using the library built in the cpp/ folder
  set(pcl_labeler_cpp_DIR "${PCL_LABELER_CPP}/install/lib/cmake/pcl_labeler_cpp")
  find_package(pcl_labeler_cpp REQUIRED PATHS ${pcl_labeler_cpp_DIR} NO_DEFAULT_PATH)
else()
  find_package(pcl_labeler_cpp REQUIRED)
endif()

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
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
  message_filters
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(${COMPONENT_NAME} pcl_labeler_cpp::pcl_labeler_cpp ${PCL_LIBRARIES})
rclcpp_components_register_node(${COMPONENT_NAME} PLUGIN "pcl_labeler::PCLLabelerNode" EXECUTABLE ${NODE_NAME})

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
