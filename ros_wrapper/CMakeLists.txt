cmake_minimum_required(VERSION 3.8)
project(floam_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../include
  ${PCL_INCLUDE_DIRS}
  "usr/include/eigen3"
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

set(LIBS 
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../lib/libfloam_core.so
  -lboost_system
)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(floam_ros src/ros_node.cpp)
ament_target_dependencies(floam_ros ${dependencies})
target_link_libraries(floam_ros ${LIBS})

install(TARGETS floam_ros DESTINATION lib/${PROJECT_NAME})