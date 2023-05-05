cmake_minimum_required(VERSION 3.0.2)
project(master_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(master src/master.cpp)
target_link_libraries(master ${catkin_LIBRARIES})
add_dependencies(master joy_node)
