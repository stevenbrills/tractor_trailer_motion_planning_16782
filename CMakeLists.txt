cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ct_optcon REQUIRED)
# find_library(ct_optcon INTERFACE)

#file(GLOB_RECURSE SRC_FILES src/*.cpp)
project(tractor_trailer_planner VERSION 1.0)
add_executable(forward_simulator src/forward_simulator.cpp)
add_executable(controller src/controller.cpp)


target_include_directories(forward_simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(forward_simulator ct_optcon)
target_link_libraries(controller ct_optcon)
# target_include_directories(forward_simulator PUBLIC ${ct_optcon_TARGET_INCLUDE_DIRS})