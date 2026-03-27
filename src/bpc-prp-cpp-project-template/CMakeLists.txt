cmake_minimum_required(VERSION 3.20)
project(prp_project)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# -----------------------------
# Motor Node
# -----------------------------
add_executable(motor_node
        src/motor_node.cpp
)

ament_target_dependencies(motor_node
        rclcpp
        std_msgs
)

install(TARGETS motor_node
        DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Line Node
# -----------------------------
add_executable(line_node
        src/line_node.cpp
)

ament_target_dependencies(line_node
        rclcpp
        std_msgs
)

install(TARGETS line_node
        DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Line Follower Node (PID)
# -----------------------------
add_executable(line_follower_node
        src/line_follower_main.cpp
)

target_include_directories(line_follower_node PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(line_follower_node
        rclcpp
        std_msgs
)

install(TARGETS line_follower_node
        DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Finalize
# -----------------------------
ament_package()