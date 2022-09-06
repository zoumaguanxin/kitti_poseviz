cmake_minimum_required(VERSION 2.6)
project(poseviz)

find_package(PCL 1.2 REQUIRED)

find_package(Eigen3 REQUIRED)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories( ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})


add_executable(poseviz src/draw_trajectory.cpp)
target_link_libraries(poseviz ${PCL_LIBRARIES})

add_executable(transformpose src/transformPose.cpp)
target_link_libraries(transformpose ${PCL_LIBRARIES})


install(TARGETS poseviz RUNTIME DESTINATION bin)

add_subdirectory(src)
add_subdirectory(include)
