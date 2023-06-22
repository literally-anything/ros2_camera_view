cmake_minimum_required(VERSION 3.8)
project(camera_view)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(camera_view src/camera_view.cpp include/camera_view/camera_view.hpp)
ament_target_dependencies(camera_view
        rclcpp
        image_transport
        cv_bridge
        OpenCV
        sensor_msgs)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(TARGETS camera_view
        DESTINATION lib/${PROJECT_NAME})

ament_package()
