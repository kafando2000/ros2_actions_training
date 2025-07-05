cmake_minimum_required(VERSION 3.8)
project(actions_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_rob_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable( count_until_server src/action_server_cpp.cpp)
add_executable(count_until_client src/action_client_cpp.cpp)
add_executable(count_until_server_queue src/action_server_cpp_queue.cpp)
add_executable(move_to_client src/move_to_client.cpp)
add_executable(move_to_server src/move_to_server.cpp)
ament_target_dependencies(count_until_server
  rclcpp
  my_rob_interfaces
  rclcpp_action
)
ament_target_dependencies(count_until_client
  rclcpp
  my_rob_interfaces
  rclcpp_action
)
ament_target_dependencies(count_until_server_queue
  rclcpp
  my_rob_interfaces
  rclcpp_action
)
ament_target_dependencies(move_to_client
  rclcpp
  my_rob_interfaces
  rclcpp_action
)
ament_target_dependencies(move_to_server
  rclcpp
  my_rob_interfaces
  rclcpp_action
)
install(TARGETS 
  count_until_server
  count_until_client
  count_until_server_queue
  move_to_client
  move_to_server
  DESTINATION lib/${PROJECT_NAME}/
)
ament_package()
