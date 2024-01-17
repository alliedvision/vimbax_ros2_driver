cmake_minimum_required(VERSION 3.8)
project(vimbax_camera)

set(VMB_DIR "" CACHE STRING "Path to VimbaX installation")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(vimbax_camera_node_SRCS
        src/vimbax_camera_node.cpp
        src/loader/library_loader_unix.cpp
        src/loader/vmbc_api.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(vimbax_camera_msgs REQUIRED)

add_executable(vimbax_camera_node ${vimbax_camera_node_SRCS})
target_include_directories(vimbax_camera_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(vimbax_camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        vimbax_camera_node
        "rclcpp"
        "image_transport"
        "vimbax_camera_msgs"
)

include_directories(${VMB_DIR}/api/include)

install(TARGETS vimbax_camera_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)
endif()

ament_package()
