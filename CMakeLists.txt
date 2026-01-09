cmake_minimum_required(VERSION 3.8)
project(gesture_legs_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)



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

include_directories(./include/gesture_legs_control/ ${Python3_INCLUDE_DIRS})

add_executable(gesture_legs_control 
    ./src/main.cpp
    ./src/gesture_control_node.cpp
    ./src/RobotLegs.cpp

)

target_link_libraries(gesture_legs_control
    pthread  # wiringPi通常需要pthread库
    ${Python3_LIBRARIES}
)

ament_target_dependencies(gesture_legs_control            #添加节点的依赖
    rclcpp
    ai_msgs
)				

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS
        gesture_legs_control
        DESTINATION lib/${PROJECT_NAME})

