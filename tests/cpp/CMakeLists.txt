cmake_minimum_required(VERSION 3.16)
project(test_node)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(test_api REQUIRED)

add_executable(test_node main.cpp)
target_link_libraries(test_node rclcpp::rclcpp rclcpp_action::rclcpp_action test_api::test_api)
