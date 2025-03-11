cmake_minimum_required(VERSION 3.8)
project(fr3_controllers)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/joint_impedance_controller.cpp
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        sensor_msgs
        franka_semantic_components
        moveit_core
        moveit_msgs
        trajectory_msgs
        geometric_shapes
)

pluginlib_export_plugin_description_file(
        controller_interface fr3_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)

ament_export_libraries(
        ${PROJECT_NAME}
)

ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        moveit_core
)

install(DIRECTORY
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()