cmake_minimum_required(VERSION 3.1)
project(simple_planner)

set(CMAKE_CXX_STANDARD 11)

#############################
#Dependencies
#############################

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    nav_msgs
    geometry_msgs
    visualization_msgs
)
catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(local_planner src/local_planner.cpp)
target_link_libraries(local_planner ${catkin_LIBRARIES})

add_executable(global_planner src/global_planner.cpp)
target_link_libraries(global_planner ${catkin_LIBRARIES})

add_executable(pid_planner src/pid_planner.cpp)
target_link_libraries(pid_planner ${catkin_LIBRARIES})
