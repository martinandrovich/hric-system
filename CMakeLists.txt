cmake_minimum_required(VERSION 3.10.2)

# project name
project(hrc-system)

# project settings
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# warning levels
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic")

# source folder (.cpp + .h files)
file(GLOB SOURCES src/*.cpp)
file(GLOB SOURCES src/modules/*.cpp)
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/src/modules/)

# executable
add_executable(${PROJECT_NAME} src/main.cpp ${SOURCES})

# dependencies

## libfranka
find_package(Franka REQUIRED)
include_directories(${FRANKA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} /opt/ros/melodic/lib/libfranka.so)