cmake_minimum_required(VERSION 3.16)
project(franka_python_bindings VERSION 0.1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

if(NOT pybind11_FOUND)
    find_package(pybind11 MODULE REQUIRED)
endif()

find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation Net)
find_package(pinocchio REQUIRED)

add_subdirectory(libfranka)

# Set the output directory for the Python module
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pylibfranka)

# Create Python module
pybind11_add_module(_pylibfranka
    src/pylibfranka.cpp
)

# Include directories
target_include_directories(_pylibfranka PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libfranka/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libfranka/common/include
)

# Link against the shared libfranka library
target_link_libraries(_pylibfranka PRIVATE
    Franka::Franka
    Eigen3::Eigen
    Poco::Foundation
    Poco::Net
    pinocchio::pinocchio
)

# Install the Python module
install(TARGETS _pylibfranka DESTINATION pylibfranka)
