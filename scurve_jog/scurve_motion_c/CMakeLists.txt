cmake_minimum_required(VERSION 3.5)

project(scurve_motion_c LANGUAGES C)

set(CMAKE_C_STANDARD 99)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(PROJECT_SOURCES
    main.c
    scurve_motion.h
    scurve_motion.c
)

add_executable(scurve_motion_c ${PROJECT_SOURCES})

# Link the math library
target_link_libraries(scurve_motion_c -lm)
