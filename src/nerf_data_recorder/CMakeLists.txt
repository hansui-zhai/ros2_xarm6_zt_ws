cmake_minimum_required(VERSION 3.8)
project(nerf_data_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
set(OpenCV_DIR /lib/x86_64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_executable(image_and_tf_recorder src/cvtfrecorder.cpp)
ament_target_dependencies(image_and_tf_recorder rclcpp sensor_msgs tf2_ros cv_bridge OpenCV nlohmann_json)
install(TARGETS image_and_tf_recorder DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch urdf rviz meshes
  DESTINATION share/nerf_data_recorder
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
