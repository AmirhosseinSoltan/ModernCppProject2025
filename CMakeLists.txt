
cmake_minimum_required(VERSION 3.31)
project(occupancy_mapper LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable folders in IDEs like VS Code 
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(Open3D REQUIRED HINTS "${CMAKE_SOURCE_DIR}/dependancies/open3d-install/lib/cmake/Open3D")
find_package(cxxopts REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(dataloader)
target_sources(dataloader PRIVATE src/dataloader.cpp)
target_include_directories(dataloader PUBLIC include/)
target_link_libraries(dataloader PUBLIC 
                                        Eigen3::Eigen 
                                        Open3D::Open3D)

add_library(bresenham)
target_sources(bresenham PRIVATE src/bresenham.cpp)
target_include_directories(bresenham PUBLIC include/)

target_link_libraries(bresenham PUBLIC dataloader)


add_executable(occupancy_mapper)
target_sources(occupancy_mapper PRIVATE src/main.cpp)
target_include_directories(occupancy_mapper PUBLIC include/)

target_link_libraries(occupancy_mapper PRIVATE 
                                                Eigen3::Eigen
                                                cxxopts::cxxopts
                                                bresenham
                                                dataloader)
