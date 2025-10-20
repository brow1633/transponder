cmake_minimum_required(VERSION 3.8)
project(transponder2ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find GeographicLib
# Append to CMAKE_MODULE_PATH since debian/ubuntu installs
# FindGeographicLib.cmake in a nonstand location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(transponder_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# git hash and branch
execute_process(
    COMMAND git describe --abbrev=7 --dirty --always --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
add_definitions(-DGIT_INFO="${GIT_BRANCH} - ${GIT_VERSION}")

# Includes
include_directories(
    include
)

# Add executables
add_executable(transponder2ros_node src/transponder2ros_node.cpp
    src/transponder2ros/functions_ros.cpp
    src/transponder2ros/functions_serial.cpp
    src/transponder2ros/functions_udp.cpp
    src/transponder2ros/transponder2ros.cpp
    )
    ament_target_dependencies(transponder2ros_node 
        rclcpp
        std_msgs
        transponder_msgs
    )
    install(TARGETS
      transponder2ros_node
        DESTINATION lib/${PROJECT_NAME}
    )

add_executable(transponder2NavSatFix_node src/transponder2NavSatFix_node.cpp
    src/transponder2NavSatFix_node.cpp
    )
    ament_target_dependencies(transponder2NavSatFix_node 
        rclcpp
        std_msgs
        sensor_msgs
        transponder_msgs
    )
    install(TARGETS
        transponder2NavSatFix_node
        DESTINATION lib/${PROJECT_NAME}
    )

add_executable(debug_node src/debug_node.cpp)
    ament_target_dependencies(debug_node 
        rclcpp
        std_msgs
        transponder_msgs
    )
    install(TARGETS
      debug_node
        DESTINATION lib/${PROJECT_NAME}
    )

add_executable(odom2transponder_node src/odom2transponder_node.cpp)
    ament_target_dependencies(odom2transponder_node 
        rclcpp
        std_msgs
        nav_msgs
        geographic_msgs
        transponder_msgs
        geometry_msgs
        GeographicLib
        tf2
        tf2_geometry_msgs
    )
    install(TARGETS
    odom2transponder_node
        DESTINATION lib/${PROJECT_NAME}
    )

# Copy over config and launch files
install(DIRECTORY
  # config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
