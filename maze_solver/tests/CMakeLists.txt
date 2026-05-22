cmake_minimum_required(VERSION 3.20)

include_directories(../include)

find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)
enable_testing()

# Pôvodné makro - linkuje všetky zdrojáky (${SRC}).
# Musí linkovať aj opencv_aruco, lebo camera_node.cpp ho používa.
macro(make_units_test name)
    add_executable(${name} t_${name}.cpp ${SRC})
    target_link_libraries(${name} GTest::GTest GTest::Main ${OpenCV_LIBS} opencv_aruco ${SDL2_LIBS})
    ament_target_dependencies(${name} rclcpp std_msgs sensor_msgs cv_bridge image_transport visualization_msgs)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

# Nové makro pre header-only triedy (napr. PlanarImuIntegrator).
# Nelinkuje ${SRC}, nepotrebuje ani OpenCV ani ROS - len gtest.
macro(make_header_only_test name)
    add_executable(${name} t_${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

make_header_only_test(planar_imu_integrator)