cmake_minimum_required(VERSION 3.8)
project(vimbax_camera_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(vimbax_camera_msgs REQUIRED)
find_package(vimbax_camera_events REQUIRED)

add_executable(asynchronous_grab_cpp src/asynchronous_grab.cpp)
ament_target_dependencies(
        asynchronous_grab_cpp
        "rclcpp"
        "image_transport"
        "vimbax_camera_msgs"
)

add_executable(event_viewer_cpp src/event_viewer.cpp)
ament_target_dependencies(
        event_viewer_cpp
        "rclcpp"
        "image_transport"
        "vimbax_camera_msgs"
        "vimbax_camera_events"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS asynchronous_grab_cpp DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_package()
