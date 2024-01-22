ament_add_gtest(${PROJECT_NAME}_vmbc_load_test
        vmbc_load_test.cpp
        APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
        ENV VIMBA_X_HOME=${VMB_DIR}
)
ament_target_dependencies(
        ${PROJECT_NAME}_vmbc_load_test
        rclcpp
)
target_link_libraries(
        ${PROJECT_NAME}_vmbc_load_test
        ${PROJECT_NAME}
)