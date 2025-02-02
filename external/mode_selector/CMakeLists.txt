cmake_minimum_required(VERSION 3.8)
project(mode_selector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

# include_directories(include)

add_executable(${PROJECT_NAME} 
  src/selector.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  std_msgs 
  autoware_auto_control_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  # config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()