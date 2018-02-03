cmake_minimum_required(VERSION 3.8)
project(lesson3_prediction)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
    classifier.cpp
    classifier.h
    main.cpp)

add_executable(lesson3_prediction ${SOURCE_FILES})