cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    rosidl_default_generators
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StringChange.srv"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker "rclcpp" "std_msgs" "geometry_msgs" "tf2" "tf2_ros")

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener "rclcpp" "std_msgs")

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(talker "${cpp_typesupport_target}")

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Add a gtest executable
  set(TESTFILES
    test/test_pub.cpp
    talker)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

  target_link_libraries(${PROJECT_NAME}_test "${cpp_typesupport_target}")
  ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  INSTALL(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
