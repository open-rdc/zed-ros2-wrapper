cmake_minimum_required(VERSION 3.8)
project(zed_image_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  # Disable pedantic warnings for ZED SDK compatibility
  add_compile_options(-Wno-pedantic -Wno-deprecated-copy -Wno-unused-parameter)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find ZED SDK
find_package(ZED 5 REQUIRED)
find_package(CUDA REQUIRED)

# Include directories
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

# Add ZED camera node
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/zed_camera_node.cpp
  src/zed_camera_component.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${ZED_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${CUDA_CUDART_LIBRARY}
)

# Register component
rclcpp_components_register_nodes(${PROJECT_NAME}_node 
  "zed_image_publisher::ZedCameraNode")

# Add executable
ament_auto_add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_node
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()