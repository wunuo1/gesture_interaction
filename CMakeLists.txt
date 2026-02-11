cmake_minimum_required(VERSION 3.8)
project(gesture_interaction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(lighting_control REQUIRED)
find_package(servo_control REQUIRED)

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

include_directories(./include/gesture_interaction/ ${Python3_INCLUDE_DIRS})

add_executable(gesture_interaction 
    ./src/main.cpp
    ./src/gesture_control_node.cpp
    ./src/actuators_control.cpp
)

add_executable(function_call_control 
    ./src/fc_call_node.cpp
    ./src/actuators_control.cpp
)

target_link_libraries(gesture_interaction
    pthread
    ${Python3_LIBRARIES}
)

ament_target_dependencies(gesture_interaction            #添加节点的依赖
    rclcpp
    ai_msgs
    lighting_control
    servo_control
)				

ament_target_dependencies(function_call_control            #添加节点的依赖
    rclcpp
    std_msgs
    lighting_control
    servo_control
)		

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS
        gesture_interaction
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        function_call_control
        DESTINATION lib/${PROJECT_NAME})

