find_package(ament_cmake_pytest REQUIRED)

set(_pytest_tests
    test_features.py
    test_streaming.py
    test_pixel_formats.py
    test_events.py
    test_load_save_settings.py
)

foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 240
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endforeach()
