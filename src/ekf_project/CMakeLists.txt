cmake_minimum_required(VERSION 3.8)
project(ekf_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---------------------------------------------------------
# 2. C++ EXECUTABLES (To be added on Day 2)
# You will add your 'add_executable' lines here tomorrow.
# ---------------------------------------------------------
include_directories(include)
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 2. Build the Odom Publisher Node
add_executable(odom_publisher src/odom_publisher.cpp)
add_executable(landmark_extractor src/landmark_extractor.cpp)
ament_target_dependencies(landmark_extractor rclcpp sensor_msgs visualization_msgs)
ament_target_dependencies(odom_publisher rclcpp geometry_msgs nav_msgs tf2_ros tf2_eigen)

# ---------------------------------------------------------
# 3. INSTALL DIRECTIVES (Crucial for Launch & Sim)
# This copies your folders to install/share/ekf_project/
# ---------------------------------------------------------

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install model files (SDF, meshes)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install config and rviz files (if you created these folders)
install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS landmark_extractor
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analyse_ekf_error.py
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)
# ---------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()