cmake_minimum_required(VERSION 3.0.2)
project(master_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp joy)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES})
