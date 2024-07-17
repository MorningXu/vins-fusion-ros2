cmake_minimum_required(VERSION 3.6)
project(vins)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_models REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(vins_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ../camera_models/include)
link_directories(../../install/camera_models/lib)

add_library(vins_lib SHARED
        src/estimator/parameters.cpp
        src/estimator/estimator.cpp
        src/estimator/feature_manager.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/projectionTwoFrameOneCamFactor.cpp
        src/factor/projectionTwoFrameTwoCamFactor.cpp
        src/factor/projectionOneFrameTwoCamFactor.cpp
        src/factor/marginalization_factor.cpp
        src/utility/utility.cpp
        src/utility/visualization.cpp
        src/utility/CameraPoseVisualization.cpp
        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp
        src/initial/initial_sfm.cpp
        src/initial/initial_ex_rotation.cpp
        src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib camera_models ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(vins_lib rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport pcl_conversions vins_msgs)


link_directories(../../install/vins/lib)
add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node camera_models vins_lib)
ament_target_dependencies(vins_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport pcl_conversions vins_msgs)

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
ament_target_dependencies(kitti_odom_test rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport pcl_conversions vins_msgs)
target_link_libraries(kitti_odom_test camera_models vins_lib)
#
add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
ament_target_dependencies(kitti_gps_test rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge camera_models image_transport pcl_conversions vins_msgs)
target_link_libraries(kitti_gps_test camera_models vins_lib)

# Install
# Install launch files
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(TARGETS vins_lib
        DESTINATION lib)

install(
        TARGETS vins_node
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS kitti_odom_test
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS kitti_gps_test
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../config
        DESTINATION ..)

ament_package()

