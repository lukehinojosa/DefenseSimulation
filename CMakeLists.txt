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
# Phase 3: Linux IPC telemetry library (POSIX shm + UDP). Unix-only.
# ---------------------------------------------------------------------------
set(SIM_HAS_IPC OFF)
if(UNIX)
    set(SIM_HAS_IPC ON)
    add_library(sim_ipc
        src/SharedMemoryChannel.cpp
        src/UdpTelemetry.cpp
    )
    target_include_directories(sim_ipc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(sim_ipc PUBLIC sim_core Threads::Threads)
    # librt provides shm_open on older glibc; harmless where it lives in libc.
    include(CheckLibraryExists)
    check_library_exists(rt shm_open "" HAVE_LIBRT)
    if(HAVE_LIBRT)
        target_link_libraries(sim_ipc PUBLIC rt)
    endif()
    target_compile_options(sim_ipc PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Demonstration executable
# ---------------------------------------------------------------------------
add_executable(defense_sim src/main.cpp)
target_link_libraries(defense_sim PRIVATE sim_core)
if(SIM_HAS_IPC)
    # Enables the "telemetry" publish mode in the demo (guarded by __linux__).
    target_link_libraries(defense_sim PRIVATE sim_ipc)

    # Secondary monitoring process (consumes the shared-memory stream).
    add_executable(telemetry_monitor src/telemetry_monitor.cpp)
    target_link_libraries(telemetry_monitor PRIVATE sim_ipc)
endif()

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

    set(SIM_TEST_SOURCES
        tests/test_vector3.cpp
        tests/test_entity.cpp
        tests/test_boundingbox.cpp
        tests/test_octree.cpp
        tests/test_engine.cpp
        tests/test_guidance.cpp
        tests/test_engagement.cpp
    )
    if(SIM_HAS_IPC)
        list(APPEND SIM_TEST_SOURCES tests/test_telemetry.cpp)
    endif()

    add_executable(sim_tests ${SIM_TEST_SOURCES})
    target_link_libraries(sim_tests PRIVATE sim_core GTest::gtest_main)
    if(SIM_HAS_IPC)
        target_link_libraries(sim_tests PRIVATE sim_ipc)
    endif()

    include(GoogleTest)
    gtest_discover_tests(sim_tests)
endif()
