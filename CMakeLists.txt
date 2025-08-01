cmake_minimum_required(VERSION 3.14)
project(OccupancyMapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable folders in IDEs like VS Code 
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# Open3D configuration - add Open3D as a subdirectory
# add_subdirectory(${CMAKE_SOURCE_DIR}/dependancies/Open3D)
# Incorporates a subdirectory containing a CMakeLists.txt into the build process, treating it as part of the project.

set(Open3D_DIR "${CMAKE_SOURCE_DIR}/dependancies/Open3D/build")
find_package(Open3D REQUIRED)

# Create the main executable
add_executable(occupancy_mapper
    src/main.cpp
)

message("CMAKE_SOURCE_DIR is: ${CMAKE_SOURCE_DIR}")
# CMAKE_SOURCE_DIR : /Users/amir/Desktop/class/4th Semester/ModernC++/project
# Include directories
target_include_directories(occupancy_mapper
    PRIVATE
    ${CMAKE_SOURCE_DIR}/dependancies/eigen-master
    ${Open3D_INCLUDE_DIRS}
    include
)

# Link dependencies
target_link_libraries(occupancy_mapper
    PRIVATE
    Open3D::Open3D
)
