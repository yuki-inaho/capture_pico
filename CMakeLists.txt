cmake_minimum_required(VERSION 3.5.1)

# project set
set(PROJ_NAME capture_flexx)
project(${PROJ_NAME})
set(SOURCE_CODE
        src/main.cpp
        src/PicoSensorClass.cpp
        src/PicoSensorSetter.cpp
        src/ParameterManager.cpp
        src/SensorManager.cpp
    )

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        rosbag
        cv_bridge
        std_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy rosbag cv_bridge std_msgs 
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(capture_flexx ${SOURCE_CODE})

add_dependencies(capture_flexx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(capture_flexx
        ${catkin_LIBRARIES}
        )

find_package(PkgConfig REQUIRED)

pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${OpenCV_CFLAGS})
target_link_libraries(${PROJ_NAME} ${OpenCV_LDFLAGS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})

pkg_check_modules(Royale libroyale REQUIRED)
include_directories(${Royale_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} ${Royale_LDFLAGS})
