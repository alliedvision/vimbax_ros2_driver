cmake_minimum_required(VERSION 3.8)
project(vimbax_camera_events)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vimbax_camera_msgs REQUIRED)


add_library(vimbax_camera_events src/vimbax_camera_events.cpp
  src/event_publisher_base.cpp
  src/event_subscriber_base.cpp)
target_compile_features(vimbax_camera_events PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(vimbax_camera_events PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  vimbax_camera_events
  "rclcpp"
  "vimbax_camera_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(vimbax_camera_events PRIVATE "VIMBAX_CAMERA_EVENTS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS vimbax_camera_events
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(
  include
)
ament_export_libraries(
  vimbax_camera_events
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
