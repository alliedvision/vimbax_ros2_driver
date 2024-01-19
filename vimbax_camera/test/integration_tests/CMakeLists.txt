ament_add_gtest(${PROJECT_NAME}_vmbc_load_test
        vmbc_load_test.cpp
        ../../src/loader/library_loader_unix.cpp
        ../../src/loader/vmbc_api.cpp
        APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
        ENV VIMBA_X_HOME=${VMB_DIR}
)
ament_target_dependencies(
        ${PROJECT_NAME}_vmbc_load_test
        rclcpp
)
target_include_directories(${PROJECT_NAME}_vmbc_load_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)