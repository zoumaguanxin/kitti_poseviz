cmake_minimum_required(VERSION 2.6)
project(poseviz)

add_executable(poseviz main.cpp)

install(TARGETS poseviz RUNTIME DESTINATION bin)
