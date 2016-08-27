cmake_minimum_required(VERSION 2.6)
project(autonomousrccar)

add_executable(autonomousrccar main.cpp)

install(TARGETS autonomousrccar RUNTIME DESTINATION bin)
