cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#find_package(car_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# message(STATUS ${rclcpp_INCLUDE_DIRS})  #ͷ�ļ���rclcpp������ͷ�ļ�
# message(STATUS ${rclcpp_LIBRARIES})  #���ļ���rclcpp�����Ŀ��ļ�

add_executable(laser_go src/laser_go.cpp)
add_executable(file_reader_cpp src/file_reader_cpp.cpp)
add_executable(txt_reader src/txt_reader.cpp)

ament_target_dependencies(laser_go rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(file_reader_cpp rclcpp std_msgs geometry_msgs)
ament_target_dependencies(txt_reader rclcpp std_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  laser_go
  file_reader_cpp
  txt_reader
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
