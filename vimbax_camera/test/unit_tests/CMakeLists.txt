
add_library(test_lib SHARED ../unit_tests/test_lib.c)
set(LIBRARY_LOADER_SRCS
        ${CMAKE_SOURCE_DIR}/src/loader/library_loader_unix.cpp
)
ament_add_gtest(${PROJECT_NAME}_library_loader_test
        library_loader_test.cpp
        ${LIBRARY_LOADER_SRCS}
        APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
)
ament_target_dependencies(
        ${PROJECT_NAME}_library_loader_test
        rclcpp
)
target_include_directories(${PROJECT_NAME}_library_loader_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
add_dependencies(${PROJECT_NAME}_library_loader_test test_lib)

ament_add_gmock(${PROJECT_NAME}_api_loader_test
        api_loader_test.cpp
        api_mock/api_mock.cpp
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