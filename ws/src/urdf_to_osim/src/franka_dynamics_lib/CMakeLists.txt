cmake_minimum_required(VERSION 2.8.3)
project(franka_dynamics)

# C++17
set(CMAKE_CXX_STANDARD 17)

# optimize library built for speed
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

# output path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# find Eigen3
set(Eigen3_INCLUDE_DIR include/eigen3)

# source files
file(GLOB SOURCES src/*.cpp)
set(SOURCES 
	src/mass.cpp
	# src/coriolis.cpp
	src/gravity.cpp
	src/friction.cpp
)

# include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${Eigen3_INCLUDE_DIR})

# add library
add_library(${PROJECT_NAME} STATIC ${SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

# install library and library headers
# install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
# file(GLOB HEADERS include/franka_dynamics/*.h)
# install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})