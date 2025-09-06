cmake_minimum_required(VERSION 3.8)
project(squatch_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(include)

# ESP Bridge Node
add_executable(esp_bridge_node src/esp_bridge_main.cpp src/esp_bridge_node.cpp)
ament_target_dependencies(esp_bridge_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs)
target_link_libraries(esp_bridge_node 
  nlohmann_json::nlohmann_json)

# Install executables
install(TARGETS
  esp_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()