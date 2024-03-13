cmake_minimum_required(VERSION 3.8)
project(vmbc_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_package()
