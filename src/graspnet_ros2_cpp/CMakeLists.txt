cmake_minimum_required(VERSION 3.8)
project(graspnet_ros2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(graspnet_player_cpp src/graspnet_player_cpp.cpp)
target_compile_features(graspnet_player_cpp PUBLIC cxx_std_17)
target_include_directories(graspnet_player_cpp PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(graspnet_player_cpp rclcpp sensor_msgs)
target_link_libraries(graspnet_player_cpp ${OpenCV_LIBS})

install(TARGETS graspnet_player_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
