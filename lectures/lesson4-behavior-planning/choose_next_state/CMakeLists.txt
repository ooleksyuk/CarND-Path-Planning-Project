cmake_minimum_required(VERSION 3.9)
project(choose_next_state)

set(CMAKE_CXX_STANDARD 11)

add_executable(choose_next_state
    cost.cpp
    cost.h
    main.cpp
    road.cpp
    road.h
    vehicle.cpp
    vehicle.h)
