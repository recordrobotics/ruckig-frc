cmake_minimum_required(VERSION 3.28)
project(ruckig_jni)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    ruckig
    GIT_REPOSITORY https://github.com/pantor/ruckig.git
    GIT_TAG        v0.15.3
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ruckig)

# Platform detection and cross-compiling setup
if (WIN32)
    set(PLATFORM_DIR "windows/x86-64/shared")
elseif(APPLE)
    set(PLATFORM_DIR "osx/universal/shared")
else()
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
        set(PLATFORM_DIR "linux/x86-64/shared")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        set(PLATFORM_DIR "linux/arm64/shared")
        message(STATUS "Cross-compiling for ARM64 Linux")
        # Cross-compiling for arm64
        if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
            set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/toolchains/arm64-toolchain.cmake" CACHE FILEPATH "Toolchain file for ARM64 cross-compiling")
        endif()
        # Ensure -fPIC for arm64
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
        # Ensure ruckig is built with PIC
        if(TARGET ruckig)
            set_target_properties(ruckig PROPERTIES POSITION_INDEPENDENT_CODE ON)
        endif()
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "athena")
        set(PLATFORM_DIR "linux/athena/shared")
        message(STATUS "Cross-compiling for Athena (FRC RoboRIO)")
        # Cross-compiling for athena
        if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
            set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/toolchains/athena-toolchain.cmake" CACHE FILEPATH "Toolchain file for Athena cross-compiling")
        endif()
        # Ensure -fPIC for athena
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
        # Ensure ruckig is built with PIC
        if(TARGET ruckig)
            set_target_properties(ruckig PROPERTIES POSITION_INDEPENDENT_CODE ON)
        endif()
    else()
        message(FATAL_ERROR "Unknown Linux platform: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()

add_library(ruckig_jni SHARED ruckig_jni.cpp)

target_include_directories(ruckig_jni PRIVATE ${JNI_INCLUDE_DIRS})
# Use the ruckig include dir from the root project
if(TARGET ruckig_frc)
    get_target_property(RUCKIG_INCLUDE_DIR ruckig_frc INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(ruckig_jni PRIVATE ${RUCKIG_INCLUDE_DIR})
endif()

target_link_libraries(ruckig_jni PRIVATE ruckig)

set(LIB_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/build/jni/${PLATFORM_DIR}")
set_target_properties(ruckig_jni PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${LIB_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${LIB_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${LIB_OUTPUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${LIB_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${LIB_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${LIB_OUTPUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR}
)

# On Windows, set the library output name to ruckig_jni.dll
if (WIN32)
    set_target_properties(ruckig_jni PROPERTIES SUFFIX ".dll")
endif()
