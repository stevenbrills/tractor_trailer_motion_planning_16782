cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "include/matplotlibcpp.h")
# set(matplotlib_cpp_DIR "../control_toolbox/matplotlib-cpp/build/")

find_package(ct_optcon REQUIRED)
# find_package(matplotlib_cpp REQUIRED)


# find_library(ct_optcon INTERFACE)

#file(GLOB_RECURSE SRC_FILES src/*.cpp)
project(tractor_trailer_planner VERSION 1.0)
# add_executable(forward_simulator src/forward_simulator.cpp src/controller.cpp)
add_executable(planner src/forward_simulator.cpp src/controller.cpp src/planner.cpp)
add_executable(collision_check_test src/main.cpp)
# add_executable(controller src/controller.cpp)
# add_executable(controller_new src/controller_new_convention.cpp)

# target_include_directories(forward_simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(collision_check_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(controller_new PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_include_directories(controller PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

# target_include_directories(controller PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})
target_link_libraries(planner ct_optcon)
# target_link_libraries(controller ct_optcon)
# target_link_libraries(controller_new ct_optcon)
# target_link_libraries(controller matplotlib_cpp)
# target_include_directories(forward_simulator PUBLIC ${ct_optcon_TARGET_INCLUDE_DIRS})