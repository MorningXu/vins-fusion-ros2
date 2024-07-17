cmake_minimum_required(VERSION 3.8)
project(sensor_pub)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(vins_lib ${CMAKE_CURRENT_BINARY_DIR}/../../install/vins/lib/vins)
message("vins_lib: " ${vins_lib})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(src ${OpenCV_INCLUDE_DIRS})

add_executable(euroc_pub_node src/euroc_pub.cpp)
target_include_directories(euroc_pub_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(euroc_pub_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(euroc_pub_node ${OpenCV_LIBS} pthread)
ament_target_dependencies(
        euroc_pub_node
        "rclcpp"
        "sensor_msgs"
        "cv_bridge"
)

install(TARGETS euroc_pub_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})


ament_package()
