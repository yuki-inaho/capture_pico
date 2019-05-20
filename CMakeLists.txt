cmake_minimum_required(VERSION 3.5.1)

# project set
set(PROJ_NAME capture_zense)
project(${PROJ_NAME})
set(SOURCE_CODE
        src/main.cpp
        src/PicoZenseSensorClass.cpp
        src/PicoZenseSetter.cpp
        src/ParameterManager.cpp
        src/SensorManager.cpp
    )

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        rosbag
        cv_bridge
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy std_msgs rosbag cv_bridge
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(capture_zense ${SOURCE_CODE})

add_dependencies(capture_zense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(capture_zense
        ${catkin_LIBRARIES}
        )

find_package(PkgConfig REQUIRED)

pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${OpenCV_CFLAGS})
target_link_libraries(${PROJ_NAME} ${OpenCV_LDFLAGS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})

pkg_check_modules(PicoZense libpicozense REQUIRED)
include_directories(${PicoZense_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${PicoZense_LDFLAGS})