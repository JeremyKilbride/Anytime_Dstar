cmake_minimum_required(VERSION 3.0)
project(Aytime_D_star)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if (NOT MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
endif()

include_directories(include)

# Add the main executable.
add_executable(planner
        src/planner.cpp src/planner_ARA.cpp src/runtest_ARA.cpp src/planner_ADstar.cpp src/runtest_ADstar.cpp)

# Define the absolute path to relevant directories.
target_compile_definitions(planner PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(planner PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
