cmake_minimum_required(VERSION 3.8)
project(lesson4_behavior_planning)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
    cost.cpp
    cost.h
    main.cpp)

add_executable(lesson4_behavior_planning ${SOURCE_FILES})