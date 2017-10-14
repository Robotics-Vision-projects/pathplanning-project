# CMake version.
cmake_minimum_required(VERSION 2.8.11)

# The name of the project.
project(rrt-path)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# The following folder will be included
include_directories("${CMAKE_SOURCE_DIR}/include")

# Use c++11 and check OpenCV 3.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(OpenCV 3 REQUIRED)

# set the RobWork root (edit in .bashrc if necessary)
set(RW_ROOT $ENV{RW_ROOT})

# Use RobWork
set(RobWork_DIR ${RW_ROOT}/cmake) 
find_package(RobWork REQUIRED)
include_directories( ${ROBWORK_INCLUDE_DIRS} )
link_directories( ${ROBWORK_LIBRARY_DIRS} )

set(SOURCEFILES src/pathplanning.cpp)

add_executable(rrt-path ${SOURCEFILES})

target_link_libraries(rrt-path ${ROBWORK_LIBRARIES})

message(${PROJECT_NAME} " done!")
