cmake_minimum_required(VERSION 3.8)
project(vimbax_camera_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(vimbax_camera_MSGS
        msg/Dummy.msg #TODO: Delete and replace with actual service definitions
)

set(vimbax_camera_SRVS
        srv/Dummy.srv #TODO: Delete and replace with actual service definitions
        srv/FeatureIntGet.srv
        srv/FeatureIntSet.srv
        srv/FeatureFloatGet.srv
        srv/FeatureFloatSet.srv
        srv/FeatureStringGet.srv
        srv/FeatureStringSet.srv 
        srv/FeatureBoolGet.srv
        srv/FeatureBoolSet.srv 
)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${vimbax_camera_MSGS}
        ${vimbax_camera_SRVS}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
