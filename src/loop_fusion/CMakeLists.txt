cmake_minimum_required(VERSION 3.8)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_models REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED common io)
find_package(pcl_conversions REQUIRED)
find_package(vins_msgs REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ../camera_models/include)
link_directories(../../install/camera_models/lib)

add_executable(loop_fusion_node
        src/pose_graph_node.cpp
        src/pose_graph.cpp
        src/keyframe.cpp
        src/utility/CameraPoseVisualization.cpp
        src/ThirdParty/DBoW/BowVector.cpp
        src/ThirdParty/DBoW/FBrief.cpp
        src/ThirdParty/DBoW/FeatureVector.cpp
        src/ThirdParty/DBoW/QueryResults.cpp
        src/ThirdParty/DBoW/ScoringObject.cpp
        src/ThirdParty/DUtils/Random.cpp
        src/ThirdParty/DUtils/Timestamp.cpp
        src/ThirdParty/DVision/BRIEF.cpp
        src/ThirdParty/VocabularyBinary.cpp
)

target_link_libraries(loop_fusion_node camera_models ${OpenCV_LIBS} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(loop_fusion_node camera_models rclcpp ament_index_cpp std_msgs sensor_msgs visualization_msgs nav_msgs cv_bridge OpenCV ament_index_cpp pcl_conversions vins_msgs)

# Install nodes
install(
        TARGETS loop_fusion_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../support_files
        DESTINATION ..)

ament_package()