cmake_minimum_required(VERSION 3.8)
project(lidar_camera_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  add_definitions(-DGALACTIC)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DHUMBLE)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the component target
ament_auto_add_library(lidar_camera_fusion_component SHARED
    src/lidar_camera_fusion_component.cpp
    src/hungarian.cpp
)
target_compile_definitions(lidar_camera_fusion_component
    PRIVATE ${PROJECT_NAME}_BUILDING_DLL)
rclcpp_components_register_nodes(lidar_camera_fusion_component
    "lidar_camera_fusion::LidarCameraFusionComponent")

# Create the executor target
ament_auto_add_executable(lidar_camera_fusion_node
    NO_TARGET_LINK_LIBRARIES
    src/lidar_camera_fusion_node.cpp
)
target_link_libraries(lidar_camera_fusion_node lidar_camera_fusion_component)

ament_auto_package()

