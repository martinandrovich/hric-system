cmake_minimum_required(VERSION 3.2)
project(pd_controller)

# C++11 config
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenSim config
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} CACHE PATH "Top-level directory of OpenSim install")
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# build
add_executable(${PROJECT_NAME} pd_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenSim_LIBRARIES})
