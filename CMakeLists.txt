
set(CMAKE_CXX_STANDARD 20)
cmake_minimum_required(VERSION 3.31)
project(occupancy_mapper LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Enable folders in IDEs like VS Code 
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Open3D configuration
set(Open3D_DIR "${CMAKE_SOURCE_DIR}/dependancies/open3d-install/lib/cmake/Open3D")
find_package(Open3D REQUIRED)

# Create the main executable
add_executable(occupancy_mapper)

target_sources(occupancy_mapper PRIVATE 
                                        src/main.cpp 
                                        src/dataloader.cpp
                                        # src/transformer.cpp
                                        src/bresenham.cpp)

target_include_directories(occupancy_mapper PRIVATE include/ ${CMAKE_SOURCE_DIR}/dependancies/eigen-master)

target_link_libraries(occupancy_mapper PRIVATE Open3D::Open3D)
