cmake_minimum_required(VERSION 3.15)
project(ceva)

include(FetchContent)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wfatal-errors")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.6
)
FetchContent_MakeAvailable(pybind11)

# Needed non-ros packages
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build ceres examples [slow]" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Do not build ceres tests [slow]" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Do no build ceres benchmarks [slow]" FORCE)
set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "Do not build ceres uninstall target")
FetchContent_Declare(
  ceres
  GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
  GIT_TAG        2.2.0
)
FetchContent_MakeAvailable(ceres)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR}
)

pybind11_add_module(
  ceva MODULE src/ceva.cpp
)

# Link the module with libraries
target_link_libraries(
  ceva PRIVATE Eigen3::Eigen fmt::fmt ${OpenMP_CXX_FLAGS} ${CERES_LIBRARIES}
)

target_compile_definitions(
  ceva PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO}
)
