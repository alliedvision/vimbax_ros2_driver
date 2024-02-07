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
        src/vimbax_camera.cpp
        src/vimbax_camera_helper.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(vimbax_camera_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED ${vimbax_camera_node_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "image_transport"
        "vimbax_camera_msgs"
)

add_executable(${PROJECT_NAME}_node src/vimbax_camera_main.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


include_directories(${VMB_DIR}/api/include)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_package()
