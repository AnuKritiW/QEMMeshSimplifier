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
include(FetchContent)

# Fetch OpenMesh
FetchContent_Declare(
    OpenMesh
    GIT_REPOSITORY https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(OpenMesh)

# OpenMesh setup
# set(OPENMESH_INCLUDE_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/OpenMesh/include)
# set(OPENMESH_LIB_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/OpenMesh/build/Build/lib)

# Fetch Polyscope
FetchContent_Declare(
    Polyscope
    GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(Polyscope)

# Polyscope setup
# set(POLYSCOPE_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/polyscope)

# Fetch Eigen
FetchContent_Declare(
    Eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(Eigen)

# Eigen setup
# set(EIGEN_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/eigen)

set(TINYFILEDIALOGS_DIR ${CMAKE_SOURCE_DIR}/dependencies/tinyfiledialogs)

#################################################################################
# Main executable: QEMSimplifier
#################################################################################

# Add the executable and its source files
add_executable(QEMSimplifier)

# find_package(OpenMesh REQUIRED)

# Source files for the main project
target_sources(QEMSimplifier PRIVATE
    src/main.cpp
    src/parser.cpp
    src/QEMSimplifier.cpp
    src/QEMSimplifierUtils.cpp
    include/parser.h
    include/QEMSimplifier.h
    include/QEMSimplifierUtils.h
    include/QEMSimplifierUtils_config.h
    ${TINYFILEDIALOGS_DIR}/tinyfiledialogs.c
)

# Include directories for the executable
target_include_directories(QEMSimplifier PRIVATE
    ${OpenMesh_SOURCE_DIR}/src
    ${Polyscope_SOURCE_DIR}/include
    ${eigen_SOURCE_DIR}
    ${TINYFILEDIALOGS_DIR}
    include
)

# # Include directories for the executable
# target_include_directories(QEMSimplifier PRIVATE
#    ${OPENMESH_INCLUDE_DIR}
#    ${OPENMESH_INCLUDE_DIR}/OpenMesh/Core
#    ${EIGEN_DIR}
#    ${TINYFILEDIALOGS_DIR}
#    include
# )

# Link OpenMesh libraries
target_link_directories(QEMSimplifier PRIVATE ${OPENMESH_LIB_DIR})
target_link_libraries(QEMSimplifier PRIVATE OpenMeshCore OpenMeshTools)

# Add Polyscope subdirectory and link Polyscope to the executable
# add_subdirectory(${POLYSCOPE_DIR} ${CMAKE_BINARY_DIR}/polyscope_build)
target_link_libraries(QEMSimplifier PRIVATE polyscope)

# Add tinyfiledialogs as a static library
add_library(tinyfiledialogs STATIC
    ${TINYFILEDIALOGS_DIR}/tinyfiledialogs.c
)
target_include_directories(tinyfiledialogs PUBLIC ${TINYFILEDIALOGS_DIR})
target_link_libraries(QEMSimplifier PRIVATE tinyfiledialogs)

###############################################################################
# Metal GPU Backend (Optional)
###############################################################################
option(USE_METAL "Use Metal GPU Backend" OFF)

if(USE_METAL)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "15.0")

  # Define a macro your code can detect
  add_definitions(-DQEM_USE_METAL)

  # Find Metal and MetalKit frameworks
  find_library(METAL_FRAMEWORK Metal)
  find_library(METALKIT_FRAMEWORK MetalKit)

  target_link_libraries(QEMSimplifier PRIVATE ${METAL_FRAMEWORK} ${METALKIT_FRAMEWORK})

  # Compile .metal to .air
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.air
    COMMAND xcrun -sdk macosx metal -std=metal3.0 -c
            ${CMAKE_CURRENT_SOURCE_DIR}/src/QEMSimplifierUtilsKernel.metal
            -o ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.air
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/QEMSimplifierUtilsKernel.metal
    COMMENT "Compiling QEMSimplifierUtilsKernel.metal to AIR"
  )

  # Link .air to .metallib
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib
    COMMAND xcrun metallib
            ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.air
            -o ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.air
    COMMENT "Linking AIR to create metallib"
  )

  # Add a custom target for the metallib
  add_custom_target(QEMSimplifierUtilsKernelMetallib
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib
  )

  # Add bridging .mm file to QEMSimplifier
  target_sources(QEMSimplifier PRIVATE src/QEMSimplifierUtils_metal.mm)

  # Compile QEMSimplifierUtils_metal.mm as Objective-C++
  set_source_files_properties(src/QEMSimplifierUtils_metal.mm PROPERTIES COMPILE_FLAGS "-x objective-c++")

  # Link the Metal frameworks to QEMSimplifier
  target_link_libraries(QEMSimplifier PRIVATE ${METAL_FRAMEWORK} ${METALKIT_FRAMEWORK})

  # Ensure main target depends on the metallib build
  add_dependencies(QEMSimplifier QEMSimplifierUtilsKernelMetallib)
  message("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
  # target_compile_definitions(QEMSimplifier PRIVATE CMAKE_METALLIB_PATH="${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib")
  # target_compile_definitions(QEMSimplifier PRIVATE CMAKE_METALLIB_PATH="${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib")
  target_compile_definitions(
    QEMSimplifier
    PRIVATE
    CMAKE_METALLIB_PATH=\"${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib\"
  )
endif()

#################################################################################
# Testing code
#################################################################################

# Fetch GoogleTest
FetchContent_Declare(
    GoogleTest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(GoogleTest)

# Set GTest paths manually
# set(GTEST_LIB_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/googletest/build/lib)
# set(GTEST_INCLUDE_DIR /Users/Exhale/Desktop/CAVE/ASE/dependencies/googletest/googletest/include)

# Find and include Google Test
# find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()

#################################################################################
# Main executable for all tests
#################################################################################

# Add the test executable + Source files for tests
add_executable(MainTests
    tests/MainTests.cpp
    tests/ParserTests.cpp
    tests/QEMSimplifierUtilsTests.cpp
    tests/QEMSimplifierTests.cpp
    src/parser.cpp
    src/QEMSimplifier.cpp
    src/QEMSimplifierUtils.cpp
)

# Include directories for tests
target_include_directories(MainTests PRIVATE
    include
    ${eigen_SOURCE_DIR}
    ${GoogleTest_SOURCE_DIR}/googletest/include
    # ${EIGEN_DIR}
    # ${GTEST_INCLUDE_DIR}
)

# Link libraries for the test executable
target_link_directories(MainTests PRIVATE ${OPENMESH_LIB_DIR} ${GTEST_LIB_DIR})
target_link_libraries(MainTests PRIVATE
    gtest
    gtest_main
    OpenMeshCore
    OpenMeshTools
)

target_compile_definitions(
      MainTests
      PRIVATE
      UNIT_TEST
)

add_test(NAME MainTests COMMAND MainTests)

#################################################################################
# Individual executables for each test file
#################################################################################

# ParserTests executable
add_executable(ParserTests
    tests/MainTests.cpp
    tests/ParserTests.cpp
    src/parser.cpp
)

target_include_directories(ParserTests PRIVATE
    include
    ${eigen_SOURCE_DIR}
    ${GoogleTest_SOURCE_DIR}/googletest/include
    # ${EIGEN_DIR}
    # ${GTEST_INCLUDE_DIR}
)

# target_link_directories(ParserTests PRIVATE ${OPENMESH_LIB_DIR} ${GTEST_LIB_DIR})
target_link_libraries(ParserTests PRIVATE
    gtest
    gtest_main
    OpenMeshCore
    OpenMeshTools
)

target_compile_definitions(ParserTests PRIVATE UNIT_TEST)
add_test(NAME ParserTests COMMAND ParserTests)

# QEMSimplifierUtilsTests executable
add_executable(QEMSimplifierUtilsTests
    tests/MainTests.cpp
    tests/QEMSimplifierUtilsTests.cpp
    src/QEMSimplifierUtils.cpp
)

target_include_directories(QEMSimplifierUtilsTests PRIVATE
    include
    ${eigen_SOURCE_DIR}
    ${GoogleTest_SOURCE_DIR}/googletest/include
    # ${EIGEN_DIR}
    # ${GTEST_INCLUDE_DIR}
)

# target_link_directories(QEMSimplifierUtilsTests PRIVATE ${OPENMESH_LIB_DIR} ${GTEST_LIB_DIR})
target_link_libraries(QEMSimplifierUtilsTests PRIVATE
    gtest
    gtest_main
    OpenMeshCore
    OpenMeshTools
)

target_compile_definitions(QEMSimplifierUtilsTests PRIVATE UNIT_TEST)
add_test(NAME QEMSimplifierUtilsTests COMMAND QEMSimplifierUtilsTests)

# QEMSimplifierTests executable
add_executable(QEMSimplifierTests
    tests/MainTests.cpp
    tests/QEMSimplifierTests.cpp
    src/parser.cpp
    src/QEMSimplifier.cpp
    src/QEMSimplifierUtils.cpp
)

target_include_directories(QEMSimplifierTests PRIVATE
    include
    ${eigen_SOURCE_DIR}
    ${GoogleTest_SOURCE_DIR}/googletest/include
    # ${EIGEN_DIR}
    # ${GTEST_INCLUDE_DIR}
)

# target_link_directories(QEMSimplifierTests PRIVATE ${OPENMESH_LIB_DIR} ${GTEST_LIB_DIR})
target_link_libraries(QEMSimplifierTests PRIVATE
    gtest
    gtest_main
    OpenMeshCore
    OpenMeshTools
)

target_compile_definitions(QEMSimplifierTests PRIVATE UNIT_TEST)
add_test(NAME QEMSimplifierTests COMMAND QEMSimplifierTests)

########


# Metal-specific configurations for tests
if(USE_METAL)
    # Include Metal bridging file
    target_sources(MainTests PRIVATE src/QEMSimplifierUtils_metal.mm)
    target_sources(QEMSimplifierUtilsTests PRIVATE src/QEMSimplifierUtils_metal.mm)
    target_sources(QEMSimplifierTests PRIVATE src/QEMSimplifierUtils_metal.mm)

    # Add dependency on the Metal library
    add_dependencies(MainTests QEMSimplifierUtilsKernelMetallib)
    add_dependencies(QEMSimplifierUtilsTests QEMSimplifierUtilsKernelMetallib)
    add_dependencies(QEMSimplifierTests QEMSimplifierUtilsKernelMetallib)

    # Link Metal and MetalKit frameworks
    target_link_libraries(MainTests PRIVATE ${METAL_FRAMEWORK} ${METALKIT_FRAMEWORK})
    target_link_libraries(QEMSimplifierUtilsTests PRIVATE ${METAL_FRAMEWORK} ${METALKIT_FRAMEWORK})
    target_link_libraries(QEMSimplifierTests PRIVATE ${METAL_FRAMEWORK} ${METALKIT_FRAMEWORK})

    # Link Objective-C runtime libraries
    find_library(FOUNDATION_FRAMEWORK Foundation)
    find_library(COREFRAMEWORK CoreFoundation)
    target_link_libraries(MainTests PRIVATE ${FOUNDATION_FRAMEWORK} ${COREFRAMEWORK})
    target_link_libraries(QEMSimplifierUtilsTests PRIVATE ${FOUNDATION_FRAMEWORK} ${COREFRAMEWORK})
    target_link_libraries(QEMSimplifierTests PRIVATE ${FOUNDATION_FRAMEWORK} ${COREFRAMEWORK})

    # Ensure Objective-C++ compilation
    set_source_files_properties(src/QEMSimplifierUtils_metal.mm PROPERTIES COMPILE_FLAGS "-x objective-c++")

    target_compile_definitions(
      MainTests PRIVATE
      UNIT_TEST
      CMAKE_METALLIB_PATH=\"${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib\"
    )
    target_compile_definitions(
        QEMSimplifierUtilsTests PRIVATE
        UNIT_TEST
        CMAKE_METALLIB_PATH=\"${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib\"
    )
    target_compile_definitions(
        QEMSimplifierTests PRIVATE
        UNIT_TEST
        CMAKE_METALLIB_PATH=\"${CMAKE_CURRENT_BINARY_DIR}/QEMSimplifierUtilsKernel.metallib\"
    )
endif()

# Automatically discover and register tests
enable_testing()
include(GoogleTest)
# Automatically discover and register tests
gtest_discover_tests(QEMSimplifierTests)
