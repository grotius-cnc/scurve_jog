cmake_minimum_required(VERSION 3.5)

project(scurve_motion LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_SOURCES
    main.cpp
    scurve_motion.h
    scurve_motion.cpp
    comments.h
)

add_executable(runtest ${PROJECT_SOURCES} )

