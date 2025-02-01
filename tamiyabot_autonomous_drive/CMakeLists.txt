cmake_minimum_required(VERSION 3.8)
project(tamiyabot_autonomous_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

pkg_check_modules(SERIAL libserial)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2_ros)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/csv_to_path.py
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)




ament_package()
