cmake_minimum_required(VERSION 3.28)
project(ruckig_frc)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Modern backend selection option
set(BACKEND "OpenGL" CACHE STRING "Graphics backend: OpenGL, DX11, or Metal")
set_property(CACHE BACKEND PROPERTY STRINGS OpenGL DX11 Metal)

message(STATUS "Compiling for graphics backend: ${BACKEND}")

include(FetchContent)

if(BACKEND STREQUAL "OpenGL" OR BACKEND STREQUAL "Metal")
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
    )
    FetchContent_MakeAvailable(glfw)
endif()

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.15.3
)
FetchContent_MakeAvailable(spdlog)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.1
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(
    ruckig
    GIT_REPOSITORY https://github.com/pantor/ruckig.git
    GIT_TAG        v0.15.3
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ruckig)

add_executable(ruckig_frc main.cpp)

target_sources(ruckig_frc
    PRIVATE
        FILE_SET CXX_MODULES FILES robot.cppm;pid.cppm;ui/uimodule.cppm;ui/ruckig.cppm;window.cppm
)

# Define MSVC deprecation warning silence macro
add_compile_definitions(_SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS)

# Enable C++23 modules if supported
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC|Clang|GNU")
    set_target_properties(ruckig_frc PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED YES)
    set_target_properties(ruckig_frc PROPERTIES CXX_EXTENSIONS OFF)
endif()

# Set compile definition for backend
string(TOUPPER "${BACKEND}" BACKEND_UPPER)
target_compile_definitions(ruckig_frc PRIVATE "BACKEND_${BACKEND_UPPER}")

# Backend-specific ImGui sources and libraries
if (BACKEND STREQUAL "OpenGL")
    set(IMGUI_BACKEND_SRC
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    set(BACKEND_LIBS glfw)
    if (WIN32)
        list(APPEND BACKEND_LIBS opengl32)
    elseif(APPLE)
        find_library(OpenGL_LIBRARY OpenGL)
        find_library(COCOA_LIBRARY Cocoa)
        find_library(IOKIT_LIBRARY IOKit)
        find_library(CoreVideo_LIBRARY CoreVideo)
        list(APPEND BACKEND_LIBS ${OpenGL_LIBRARY} ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${CoreVideo_LIBRARY})
    else()
        find_package(OpenGL REQUIRED)
        list(APPEND BACKEND_LIBS OpenGL::GL)
    endif()
elseif (BACKEND STREQUAL "DX11")
    set(IMGUI_BACKEND_SRC
        ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
    )
    set(BACKEND_LIBS d3d11 dxgi)
elseif (BACKEND STREQUAL "Metal")
    target_compile_definitions(ruckig_frc PRIVATE "IMGUI_IMPL_METAL_CPP")
    set(IMGUI_BACKEND_SRC
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_metal.mm
    )
    set(BACKEND_LIBS glfw)
    if(METAL_CPP_INCLUDE_PATH)
        target_include_directories(ruckig_frc PRIVATE
            ${METAL_CPP_INCLUDE_PATH}
        )
    else()
        message(FATAL_ERROR "METAL_CPP_INCLUDE_PATH is not set. Please set it to the path of the Metal C++ headers.")
    endif()

    target_sources(ruckig_frc PRIVATE window-metal.mm)
    # Disable warning for arc bridge casts in non-ARC files for Metal backend
    set_source_files_properties(
        ${imgui_SOURCE_DIR}/backends/imgui_impl_metal.mm
        window-metal.mm
        PROPERTIES COMPILE_FLAGS "-Wno-arc-bridge-casts-disallowed-in-nonarc"
    )

    if(APPLE)
        find_library(METAL_LIBRARY Metal)
        find_library(METALKIT_LIBRARY MetalKit)
        find_library(COCOA_LIBRARY Cocoa)
        find_library(IOKIT_LIBRARY IOKit)
        find_library(CoreVideo_LIBRARY CoreVideo)
        find_library(QuartzCore_LIBRARY QuartzCore)
        find_library(FOUNDATION_LIBRARY Foundation)
        list(APPEND BACKEND_LIBS ${METAL_LIBRARY} ${METALKIT_LIBRARY} ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${CoreVideo_LIBRARY} ${QuartzCore_LIBRARY} ${FOUNDATION_LIBRARY})
    endif()
endif()

# Common ImGui sources
set(IMGUI_SRC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${IMGUI_BACKEND_SRC}
)
target_sources(ruckig_frc PRIVATE ${IMGUI_SRC})

# ImGui include dirs
set(IMGUI_INCLUDE_DIRS
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_include_directories(ruckig_frc PRIVATE ${IMGUI_INCLUDE_DIRS})

# Link libraries
# Always link spdlog and ruckig, plus backend-specific
set(LINK_LIBS spdlog::spdlog ruckig ${BACKEND_LIBS})
target_link_libraries(ruckig_frc PRIVATE ${LINK_LIBS})

if(NOT DEFINED BUILD_OUTPUT_DIR)
    set(BUILD_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/build/output/")
endif()
set_target_properties(ruckig_frc PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BUILD_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${BUILD_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${BUILD_OUTPUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BUILD_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${BUILD_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${BUILD_OUTPUT_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${BUILD_OUTPUT_DIR}
)

# Copy font files to output directory after build
add_custom_command(TARGET ruckig_frc POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/fonts"
        $<TARGET_FILE_DIR:ruckig_frc>/fonts
    COMMENT "Copying all font files to output directory"
)
