
ament_add_gmock(${PROJECT_NAME}_api_loader_test
        api_loader_test.cpp
        ${CMAKE_SOURCE_DIR}/src/loader/vmbc_api.cpp
        ${CMAKE_SOURCE_DIR}/src/loader/library_loader_unix.cpp
)
ament_target_dependencies(
        ${PROJECT_NAME}_api_loader_test
        rclcpp
)
target_include_directories(${PROJECT_NAME}_api_loader_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)