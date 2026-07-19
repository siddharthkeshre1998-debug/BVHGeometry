cmake_minimum_required(VERSION 3.20)

project(
    GeometryKernel
    VERSION 1.0.0
    DESCRIPTION "Computational Geometry Library"
    LANGUAGES CXX
)

# --------------------------------------------------------------------
# C++ Standard
# --------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --------------------------------------------------------------------
# Compiler Warnings
# --------------------------------------------------------------------

if(MSVC)

    add_compile_options(
        /W4
        /permissive-
    )

else()

    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
    )

endif()

# --------------------------------------------------------------------
# Library
# --------------------------------------------------------------------

file(GLOB_RECURSE GEOMETRY_HEADERS
    include/*.h
)

file(GLOB_RECURSE GEOMETRY_SOURCES
    src/*.cpp
)

add_library(
    GeometryKernel
    ${GEOMETRY_HEADERS}
    ${GEOMETRY_SOURCES}
)

target_include_directories(
    GeometryKernel
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(
    GeometryKernel
    PUBLIC
        cxx_std_20
)

# --------------------------------------------------------------------
# Build Options
# --------------------------------------------------------------------

option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTS "Build tests" OFF)

# --------------------------------------------------------------------
# Examples
# --------------------------------------------------------------------

if(BUILD_EXAMPLES)

    add_subdirectory(examples)

endif()

# --------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------

if(BUILD_TESTS)

    enable_testing()

    add_subdirectory(tests)

endif()
