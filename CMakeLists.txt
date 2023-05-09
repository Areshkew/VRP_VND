cmake_minimum_required(VERSION 3.20)

project( VND_VRP )

add_subdirectory(src)

add_executable(VND_VRP main.cpp)

find_package(Matplot++ REQUIRED)

target_link_libraries(VND_VRP Node VRP Vehicle Route Matplot++::matplot QuadTree LocalSearch)