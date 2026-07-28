cmake_minimum_required(VERSION 3.8)
project(icp_localization_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-O3)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

# Let ament discover standard ROS2 components safely from package.xml
ament_auto_find_build_dependencies()

# Create target executable wrapper from src files
ament_auto_add_executable(icp_localization 
  DIRECTORY src)

# ◄ THE TRUE PLAIN SIGNATURE FIX:
# Removed the 'PRIVATE' keyword completely. Now it is a pure plain signature call
# which perfectly matches ament_target_dependencies' underlying behavior.
if(TARGET icp_localization)
  target_link_libraries(icp_localization gpu_bbs3d)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)