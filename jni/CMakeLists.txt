cmake_minimum_required(VERSION 3.28)
project(ruckig_jni)

find_package(Java REQUIRED)
find_package(JNI REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ruckig_jni SHARED ruckig_jni.cpp)

target_include_directories(ruckig_jni PRIVATE ${JNI_INCLUDE_DIRS})
# Use the ruckig include dir from the root project
if(TARGET ruckig_frc)
    get_target_property(RUCKIG_INCLUDE_DIR ruckig_frc INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(ruckig_jni PRIVATE ${RUCKIG_INCLUDE_DIR})
endif()

target_link_libraries(ruckig_jni PRIVATE ruckig)

# On Windows, set the library output name to ruckig_jni.dll
if (WIN32)
    set_target_properties(ruckig_jni PROPERTIES SUFFIX ".dll")
endif()
