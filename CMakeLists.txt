cmake_minimum_required(VERSION 3.20)
project(DefenseSimulation LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Core simulation library (Phase 1: math, entities, octree, engine)
# ---------------------------------------------------------------------------
add_library(sim_core
    src/Octree.cpp
    src/SimulationEngine.cpp
    src/EngagementManager.cpp
)
target_include_directories(sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_core PUBLIC Threads::Threads)

if(MSVC)
    target_compile_options(sim_core PRIVATE /W4)
else()
    target_compile_options(sim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Demonstration executable
# ---------------------------------------------------------------------------
add_executable(defense_sim src/main.cpp)
target_link_libraries(defense_sim PRIVATE sim_core)

# ---------------------------------------------------------------------------
# Tests (GoogleTest via FetchContent)
# ---------------------------------------------------------------------------
option(SIM_BUILD_TESTS "Build the GoogleTest suite" ON)

if(SIM_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    # Keep GoogleTest from clobbering our compiler/runtime settings on Windows.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(sim_tests
        tests/test_vector3.cpp
        tests/test_entity.cpp
        tests/test_boundingbox.cpp
        tests/test_octree.cpp
        tests/test_engine.cpp
        tests/test_guidance.cpp
        tests/test_engagement.cpp
    )
    target_link_libraries(sim_tests PRIVATE sim_core GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(sim_tests)
endif()
