# Minimum required CMake version
cmake_minimum_required(VERSION 3.2)

# Project name
project(QEMSimplifier_build)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Toolchain file handling (if defined in the environment)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

#################################################################################
# Directories and paths setup
#################################################################################

# OpenMesh setup
# set(OPENMESH_INCLUDE_DIR /home/s5647918/local/OpenMesh/include)
# set(OPENMESH_LIB_DIR /home/s5647918/local/OpenMesh/lib)

# OpenMesh setup
# set(OPENMESH_INCLUDE_DIR /usr/local/Cellar/open-mesh/11.0.0/include/OpenMesh)
# set(OPENMESH_LIB_DIR /usr/local/Cellar/open-mesh/11.0.0/lib)

# OpenMesh setup
set(OPENMESH_INCLUDE_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/OpenMesh/include)
set(OPENMESH_LIB_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/OpenMesh/build/Build/lib)

# Polyscope setup
# set(POLYSCOPE_DIR /home/s5647918/Code/dependencies/polyscope)
set(POLYSCOPE_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/polyscope)

# Eigen setup
# set(EIGEN_DIR /home/s5647918/Code/dependencies/eigen)
set(EIGEN_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/eigen)

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
)

# Include directories for the executable
target_include_directories(QEMSimplifier PRIVATE
   ${OPENMESH_INCLUDE_DIR}
   ${OPENMESH_INCLUDE_DIR}/OpenMesh/Core
   ${EIGEN_DIR}
   include
)

# Link OpenMesh libraries
target_link_directories(QEMSimplifier PRIVATE ${OPENMESH_LIB_DIR})
target_link_libraries(QEMSimplifier PRIVATE OpenMeshCore OpenMeshTools)

# Add Polyscope subdirectory and link Polyscope to the executable
add_subdirectory(${POLYSCOPE_DIR} ${CMAKE_BINARY_DIR}/polyscope_build)
target_link_libraries(QEMSimplifier PRIVATE polyscope)

#################################################################################
# Testing code
#################################################################################

# Set GTest paths manually
set(GTEST_LIB_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/googletest/build/lib)
set(GTEST_INCLUDE_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/googletest/googletest/include)

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
