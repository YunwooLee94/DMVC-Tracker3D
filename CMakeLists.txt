cmake_minimum_required(VERSION 3.0.2)
project(dmvc_tracker3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        decomp_ros_utils
        decomp_util
        geometry_msgs
        nav_msgs
        pcl_conversions
        roscpp
        std_msgs
        tf
        visualization_msgs
        )

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_message_files(FILES ControlInput.msg ControlInputList.msg ObjectState.msg ObjectStateList.msg PolyState.msg)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
        LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        std_msgs
        message_runtime
)

file(GLOB DMVC_UTILS_SRC src/dmvc_utils3d/bernstein_util.cpp src/dmvc_utils3d/Utils.cpp)
file(GLOB DMVC_SIMULATOR3D_SRC src/dmvc_simulator3d/*.cpp)
#
file(GLOB DMVC_PREDICTOR3D_SRC src/dmvc_predictor3d/Predictor.cpp src/dmvc_predictor3d/PredictionVisualizer.cpp)
file(GLOB DMVC_TRACKER3D_SRC src/dmvc_tracker3d/PlannerBase.cpp src/dmvc_tracker3d/Tracker.cpp src/dmvc_tracker3d/PlanningVisualizer.cpp src/dmvc_tracker3d/RosWrapper.cpp src/dmvc_tracker3d/Wrapper.cpp)
file(GLOB DMVC_REPLAYER3D_SRC src/dmvc_replay3d/Replayer.cpp)

add_executable(dmvc_simulator3d exe/main_simulator3d.cpp ${DMVC_SIMULATOR3D_SRC} ${DMVC_UTILS_SRC})
target_link_libraries(dmvc_simulator3d ${catkin_LIBRARIES} -lstdc++fs)

add_executable(dmvc_predictor3d exe/main_predictor3d.cpp ${DMVC_PREDICTOR3D_SRC} ${DMVC_UTILS_SRC})
target_link_libraries(dmvc_predictor3d ${catkin_LIBRARIES} -lstdc++fs)

add_executable(dmvc_tracker3d exe/main_tracker3d.cpp ${DMVC_TRACKER3D_SRC} ${DMVC_UTILS_SRC})
target_link_libraries(dmvc_tracker3d ${catkin_LIBRARIES} -lstdc++fs)

add_executable(dmvc_total3d exe/main_total3d.cpp src/dmvc_replay3d/TotalTrajReader.cpp)
target_link_libraries(dmvc_total3d ${catkin_LIBRARIES} -lstdc++fs)

add_executable(dmvc_replay3d exe/main_replay3d.cpp src/dmvc_replay3d/Replayer.cpp)
target_link_libraries(dmvc_replay3d ${catkin_LIBRARIES} -lstdc++fs)

add_executable(dmvc_camera_tf3d exe/main_camera_tf3d.cpp src/dmvc_camera_tf3d/CameraTfPublisher.cpp)
target_link_libraries(dmvc_camera_tf3d ${catkin_LIBRARIES} -lstdc++fs)