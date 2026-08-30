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
    src/MapData.cpp
)
target_include_directories(sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_core PUBLIC Threads::Threads)

if(MSVC)
    target_compile_options(sim_core PRIVATE /W4)
else()
    target_compile_options(sim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Networking library (UDP telemetry + consumer). Cross-platform: POSIX sockets
# on Unix, Winsock2 on Windows. Enables the remote visualizer on any OS.
# ---------------------------------------------------------------------------
add_library(sim_net
    src/UdpTelemetry.cpp
    src/UdpTelemetryConsumer.cpp
)
target_include_directories(sim_net PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_net PUBLIC sim_core)
if(WIN32)
    target_link_libraries(sim_net PUBLIC ws2_32)
endif()
if(MSVC)
    target_compile_options(sim_net PRIVATE /W4)
else()
    target_compile_options(sim_net PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Shared-memory telemetry library (cross-platform via Boost.Interprocess).
# POSIX shm on Unix, native file mapping on Windows -- so the shared-memory
# publisher, monitor, and "--source shm" visualizer run natively everywhere.
# ---------------------------------------------------------------------------
# Boost.Interprocess is header-only, so all we need is its include directory.
# Locating it by header keeps this independent of Boost's version and of the
# classic-vs-modular find_package differences (Debian's 1.74 vs vcpkg's 1.90).
# The vcpkg toolchain (set in the Windows presets) adds its tree to the search
# path; on Linux the system include dir is searched by default.
# A best-effort find_package first populates Boost_INCLUDE_DIRS on systems where
# it works; find_path then locates the header directly, with hints covering
# vcpkg's classic install tree.
find_package(Boost QUIET)
find_path(BOOST_INTERPROCESS_INCLUDE_DIR
    NAMES boost/interprocess/mapped_region.hpp
    HINTS
        ${Boost_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        $ENV{BOOST_ROOT}
        ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}
        ${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}
    PATH_SUFFIXES include
    DOC "Directory containing Boost.Interprocess headers")
if(NOT BOOST_INTERPROCESS_INCLUDE_DIR)
    message(FATAL_ERROR
        "Boost.Interprocess headers not found. Install 'libboost-dev' (Linux) "
        "or 'vcpkg install boost-interprocess' (Windows).")
endif()
message(STATUS "Boost.Interprocess headers: ${BOOST_INTERPROCESS_INCLUDE_DIR}")

add_library(sim_ipc
    src/SharedMemoryChannel.cpp
    src/ShmTelemetryConsumer.cpp
)
target_include_directories(sim_ipc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${BOOST_INTERPROCESS_INCLUDE_DIR})
target_link_libraries(sim_ipc PUBLIC sim_core Threads::Threads)

if(UNIX)
    # librt provides shm_open on older glibc; harmless where it lives in libc.
    include(CheckLibraryExists)
    check_library_exists(rt shm_open "" HAVE_LIBRT)
    if(HAVE_LIBRT)
        target_link_libraries(sim_ipc PUBLIC rt)
    endif()
endif()
if(MSVC)
    # /wd4324: the alignas(64) padding on the ring slots is intentional.
    target_compile_options(sim_ipc PRIVATE /W4 /wd4324)
else()
    target_compile_options(sim_ipc PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# YAML config (yaml-cpp): prefer an installed package (vcpkg on Windows), and
# FetchContent-build it from source otherwise (e.g. the Pi). The scenario
# constants live in config/simulation.yaml and are loaded at runtime.
# ---------------------------------------------------------------------------
find_package(yaml-cpp CONFIG QUIET)
if(NOT yaml-cpp_FOUND AND NOT TARGET yaml-cpp)
    include(FetchContent)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0
    )
    FetchContent_MakeAvailable(yaml-cpp)
    message(STATUS "yaml-cpp: built from source via FetchContent")
else()
    message(STATUS "yaml-cpp: using installed package")
endif()
# The exported target is namespaced in recent versions and bare in older ones.
if(TARGET yaml-cpp::yaml-cpp)
    set(SIM_YAMLCPP yaml-cpp::yaml-cpp)
else()
    set(SIM_YAMLCPP yaml-cpp)
endif()

# ---------------------------------------------------------------------------
# Demonstration executable
# ---------------------------------------------------------------------------
add_executable(defense_sim src/main.cpp src/SimConfig.cpp)
# The "telemetry" publish mode (shm + optional UDP streaming) is now available
# on every platform.
target_link_libraries(defense_sim PRIVATE sim_core sim_ipc sim_net ${SIM_YAMLCPP})

# Secondary monitoring process (consumes the shared-memory stream).
add_executable(telemetry_monitor src/telemetry_monitor.cpp)
target_link_libraries(telemetry_monitor PRIVATE sim_ipc)

# ---------------------------------------------------------------------------
# Phase 4: Raylib 3D C2 visualizer (optional; keeps headless/CI builds clean).
# ---------------------------------------------------------------------------
option(SIM_BUILD_VISUALIZER "Build the Raylib C2 visualizer" OFF)

if(SIM_BUILD_VISUALIZER)
    # Prefer an already-installed raylib (e.g. vcpkg on Windows, which ships a
    # matching header + import lib + DLL). This avoids a header/ABI clash where
    # vcpkg's raylib.h (dllimport) shadows a FetchContent build. Only fetch and
    # build raylib from source when none is installed (e.g. on the Pi).
    find_package(raylib QUIET)
    if(NOT raylib_FOUND)
        include(FetchContent)
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            raylib
            GIT_REPOSITORY https://github.com/raysan5/raylib.git
            GIT_TAG 5.5
        )
        FetchContent_MakeAvailable(raylib)
        message(STATUS "raylib: built from source via FetchContent")
    else()
        message(STATUS "raylib: using installed package ${raylib_VERSION}")
    endif()

    add_executable(c2_visualizer src/c2_visualizer.cpp)
    # Both transports (--source shm and --source udp) are now cross-platform.
    target_link_libraries(c2_visualizer PRIVATE sim_core sim_ipc sim_net raylib)
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
        tests/test_structureindex.cpp
        tests/test_engine.cpp
        tests/test_guidance.cpp
        tests/test_engagement.cpp
        tests/test_udp.cpp         # cross-platform: protocol + UDP + consumer
        tests/test_telemetry.cpp   # cross-platform shared-memory (Boost)
    )

    add_executable(sim_tests ${SIM_TEST_SOURCES})
    # sim_net (UDP) and sim_ipc (shared memory) are both cross-platform now.
    target_link_libraries(sim_tests
        PRIVATE sim_core sim_net sim_ipc GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(sim_tests)
endif()
