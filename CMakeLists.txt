# Minimum required CMake version
cmake_minimum_required(VERSION 3.2)

# Project name
project(QEMSimplifier_build)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Build architecture")

# Detect platform and architecture
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    message(STATUS "Configuring for macOS")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        set(CMAKE_OSX_ARCHITECTURES "arm64")
        message(STATUS "Targeting arm64 on macOS (Apple Silicon)")
    else()
        set(CMAKE_OSX_ARCHITECTURES "x86_64")
        message(STATUS "Targeting x86_64 on macOS (Intel)")
    endif()
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    message(STATUS "Configuring for Linux")
    # Add any architecture-specific handling for Linux if needed
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "Configuring for Windows")
    # Add any architecture-specific handling for Windows if needed
else()
    message(WARNING "Unsupported platform detected: ${CMAKE_SYSTEM_NAME}")
endif()

# Toolchain file handling (if defined in the environment)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

# Check if building inside Docker
if(DEFINED ENV{DOCKER_BUILD})
    set(IS_DOCKER_BUILD TRUE)
else()
    set(IS_DOCKER_BUILD FALSE)
endif()

#################################################################################
# Directories and paths setup
#################################################################################

set(DEPENDENCIES_DIR ${CMAKE_SOURCE_DIR}/dependencies)

# Tinyfiledialogs setup
set(TINYFILEDIALOGS_DIR ${DEPENDENCIES_DIR}/tinyfiledialogs)

if(IS_DOCKER_BUILD) # Docker dependencies (pre-installed)
    # OpenMesh setup
    set(OPENMESH_INCLUDE_DIR ${DEPENDENCIES_DIR}/OpenMesh/include)
    set(OPENMESH_LIB_DIR ${DEPENDENCIES_DIR}/OpenMesh/build/lib)

    # Polyscope setup
    set(POLYSCOPE_DIR ${DEPENDENCIES_DIR}/polyscope)
    add_subdirectory(${POLYSCOPE_DIR} ${CMAKE_BINARY_DIR}/polyscope_build)

    # Eigen setup
    set(EIGEN_DIR ${DEPENDENCIES_DIR}/eigen)

    # Google Test setup
    set(GTEST_DIR ${DEPENDENCIES_DIR}/googletest)
    set(GTEST_LIB_DIR ${GTEST_DIR}/build/lib)
    set(GTEST_INCLUDE_DIR ${GTEST_DIR}/googletest/include)
else() # Local build: Fetch dependencies dynamically
    include(FetchContent)

    # Fetch OpenMesh
    FetchContent_Declare(
        openmesh
        GIT_REPOSITORY https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
        GIT_TAG master
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(openmesh)

    # Fetch Polyscope
    FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG master
        GIT_SUBMODULES_RECURSE TRUE
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(polyscope)

    # Fetch Eigen
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(eigen)

    # Fetch GoogleTest
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG main
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(googletest)

    # Set paths for fetched dependencies
    set(OPENMESH_INCLUDE_DIR ${openmesh_SOURCE_DIR}/include)
    set(OPENMESH_LIB_DIR ${openmesh_BINARY_DIR}/build/lib)
    set(EIGEN_DIR ${eigen_SOURCE_DIR})
    set(POLYSCOPE_DIR ${polyscope_SOURCE_DIR})
    set(GTEST_INCLUDE_DIR ${googletest_SOURCE_DIR}/googletest/include)
    set(GTEST_LIB_DIR ${googletest_BINARY_DIR})
endif()

#################################################################################
# Main executable: QEMSimplifier
#################################################################################

# Add the executable and its source files
add_executable(QEMSimplifier)

find_package(OpenMesh REQUIRED)

# Source files for the main project
target_sources(QEMSimplifier PRIVATE
    src/main.cpp
    src/parser.cpp
    src/QEMSimplifier.cpp
    include/parser.h
    include/QEMSimplifier.h
    ${TINYFILEDIALOGS_DIR}/tinyfiledialogs.c
)

# Include directories for the executable
target_include_directories(QEMSimplifier PRIVATE
   ${OPENMESH_INCLUDE_DIR}
   ${OPENMESH_INCLUDE_DIR}/OpenMesh/Core
   ${EIGEN_DIR}
   ${TINYFILEDIALOGS_DIR}
   include
)

# Link OpenMesh libraries
target_link_directories(QEMSimplifier PRIVATE ${OPENMESH_LIB_DIR})
target_link_libraries(QEMSimplifier PRIVATE OpenMeshCore OpenMeshTools)

# Add Polyscope subdirectory and link Polyscope to the executable
target_link_libraries(QEMSimplifier PRIVATE polyscope)

#################################################################################
# Testing code
#################################################################################

# Set GTest paths manually
set(GTEST_DIR ${DEPENDENCIES_DIR}/googletest)
set(GTEST_LIB_DIR ${GTEST_DIR}/build/lib)
set(GTEST_INCLUDE_DIR ${GTEST_DIR}/googletest/include)

# Find and include Google Test
# find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()

# Add the test executable
add_executable(QEMSimplifierTests)

# Source files for tests
target_sources(QEMSimplifierTests PRIVATE
    tests/QEMSimplifierTests.cpp
    src/parser.cpp
    src/QEMSimplifier.cpp
)

# Include directories for tests
target_include_directories(QEMSimplifierTests PRIVATE
    include
    ${EIGEN_DIR}
    ${GTEST_INCLUDE_DIRS}
)

# Link libraries for the test executable
# target_link_directories(QEMSimplifierTests PRIVATE ${OPENMESH_LIB_DIR})
target_link_directories(QEMSimplifierTests PRIVATE ${OPENMESH_LIB_DIR} ${GTEST_LIB_DIR})
target_link_libraries(QEMSimplifierTests PRIVATE
    gtest
    gtest_main
    OpenMeshCore
    OpenMeshTools
)

# Automatically discover and register tests
enable_testing()
include(GoogleTest)
# Automatically discover and register tests
gtest_discover_tests(QEMSimplifierTests)
