cmake_minimum_required(VERSION 3.16)
project(chr_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(chr_kinematics
  src/whole_body_kinematics.cpp
  src/px4_frame_adapter.cpp)
target_include_directories(chr_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(chr_kinematics Eigen3::Eigen)

add_executable(chr_controller_node src/chr_controller_node.cpp)
target_link_libraries(chr_controller_node chr_kinematics Eigen3::Eigen)
ament_target_dependencies(
  chr_controller_node geometry_msgs px4_msgs rclcpp sensor_msgs std_msgs std_srvs)

install(TARGETS chr_kinematics chr_controller_node
  EXPORT export_chr_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_chr_kinematics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()
