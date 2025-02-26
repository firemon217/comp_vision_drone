cmake_minimum_required(VERSION 3.5)
project(dynamicframework)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(keyboard_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(comp_vision src/comp_vision/comp_vision.cpp)
target_link_libraries(comp_vision ${OpenCV_LIBS})
ament_target_dependencies(comp_vision cv_bridge rclcpp)

add_executable(dynamicframework src/main.cpp)
target_link_libraries(dynamicframework comp_vision)
ament_target_dependencies(dynamicframework rclcpp std_msgs mavros_msgs geometry_msgs sensor_msgs keyboard_msgs cv_bridge)

install(TARGETS
  dynamicframework
  DESTINATION lib/${PROJECT_NAME})

ament_package()