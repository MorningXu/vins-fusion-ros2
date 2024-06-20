cmake_minimum_required(VERSION 3.6)
project(global_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


find_package(Ceres REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
  ${CERES_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)

add_executable(global_fusion_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_fusion_node ${CERES_LIBRARIES} libGeographiccc)
ament_target_dependencies(global_fusion_node rclcpp std_msgs nav_msgs sensor_msgs visualization_msgs)

# Install nodes
install(
		TARGETS global_fusion_node
		DESTINATION lib/${PROJECT_NAME}
)

ament_package()
