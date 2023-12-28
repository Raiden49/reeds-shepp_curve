cmake_minimum_required(VERSION 3.16)
project(rs_curve DESCRIPTION "a rs curve example")

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    "src/rs_curve.cpp"
    "src/rs_curve_node.cpp"
    "src/main.cpp"
)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)