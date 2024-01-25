
add_library(test_lib SHARED ../unit_tests/test_lib.c)
ament_add_gtest(${PROJECT_NAME}_library_loader_test
        library_loader_test.cpp
        APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
)
ament_target_dependencies(
        ${PROJECT_NAME}_library_loader_test
        rclcpp
)
target_link_libraries(
        ${PROJECT_NAME}_library_loader_test
        ${PROJECT_NAME}
)
add_dependencies(${PROJECT_NAME}_library_loader_test test_lib)

ament_add_gmock(${PROJECT_NAME}_api_loader_test
        api_loader_test.cpp
        mocks/api_mock.cpp
)
ament_target_dependencies(
        ${PROJECT_NAME}_api_loader_test
        rclcpp
)
target_link_libraries(
        ${PROJECT_NAME}_api_loader_test
        ${PROJECT_NAME}
)

ament_add_gmock(${PROJECT_NAME}_vimbax_camera_test
        vimbax_camera_test.cpp
        mocks/api_mock.cpp
)
ament_target_dependencies(
        ${PROJECT_NAME}_vimbax_camera_test
        rclcpp
)
target_link_libraries(
        ${PROJECT_NAME}_vimbax_camera_test
        ${PROJECT_NAME}
)