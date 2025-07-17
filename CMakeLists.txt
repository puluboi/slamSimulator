cmake_minimum_required(VERSION 3.10.0)
project(slamSimulator VERSION 0.1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
# Add all source files
add_executable(slamSimulator 
    src/main.cpp
    src/game.cpp
    src/agent.cpp
    src/sensors.cpp
    src/odometry.cpp
    src/movement.cpp
    src/renderer.cpp
    src/pathfinding.cpp
    src/ekf.cpp
)

# Link SFML libraries
target_link_libraries(slamSimulator sfml-graphics sfml-window sfml-system)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
