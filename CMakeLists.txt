# We will always try to use a version > 3.1 if avaliable
cmake_minimum_required(VERSION 3.2)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

# Project name
project(NIT_build)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add OpenMesh installation paths
set(OPENMESH_INCLUDE_DIR /home/s5647918/local/OpenMesh/include)
set(OPENMESH_LIB_DIR /home/s5647918/local/OpenMesh/lib)

# Add the OpenMesh include directory to the include path
include_directories(${OPENMESH_INCLUDE_DIR})

# Add the executable and its source files
add_executable(NIT)
# We now add the source files we want for the project. for now we are going to just add main.cpp
# See here for more details https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/
target_sources(NIT PRIVATE src/main.cpp src/parser.cpp include/parser.h)

target_include_directories(NIT PRIVATE include)

# Link OpenMesh libraries
target_link_directories(NIT PRIVATE ${OPENMESH_LIB_DIR})
target_link_libraries(NIT PRIVATE OpenMeshCore OpenMeshTools)

#################################################################################
# Testing code
#################################################################################

# Find and include Google Test
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()

# Add the test executable
add_executable(NITTests)

# Add test source files, including parser.cpp
target_sources(NITTests PRIVATE tests/NITTests.cpp src/parser.cpp)

# Include directories for the tests
target_include_directories(NITTests PRIVATE include ${GTEST_INCLUDE_DIRS})

# Link Google Test and OpenMesh for the test executable
target_link_directories(NITTests PRIVATE ${OPENMESH_LIB_DIR})
target_link_libraries(NITTests PRIVATE GTest::gtest GTest::gtest_main OpenMeshCore OpenMeshTools)

# Automatically discover tests
gtest_discover_tests(NITTests)
