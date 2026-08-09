cmake_minimum_required(VERSION 3.22.1)
project(mask_lidar_fusion)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nlohmann_json REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(HEADERS 
    include/mask_lidar_fusion/mask_lidar_fusion.hpp
    include/mask_lidar_fusion/extrinsics_parser.hpp
)

set(SOURCES 
    src/mask_lidar_fusion.cpp
    src/extrinsics_parser.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} nlohmann_json::nlohmann_json)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Config
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)