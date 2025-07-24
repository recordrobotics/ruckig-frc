cmake_minimum_required(VERSION 3.28)
project(ruckig_sim)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.13.0
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
FetchContent_MakeAvailable(ruckig)

add_executable(ruckig_sim main.cpp)

target_sources(ruckig_sim
    PRIVATE
        FILE_SET CXX_MODULES FILES ui/uimodule.cppm;ui/ruckig.cppm;window.cppm
)

# Define MSVC deprecation warning silence macro
add_compile_definitions(_SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS)

# Enable C++23 modules if supported
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC|Clang|GNU")
    set_target_properties(ruckig_sim PROPERTIES CXX_STANDARD 23 CXX_STANDARD_REQUIRED YES)
    set_target_properties(ruckig_sim PROPERTIES CXX_EXTENSIONS OFF)
endif()

# Add ImGui include directories manually
set(IMGUI_INCLUDE_DIRS
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_include_directories(ruckig_sim PRIVATE ${IMGUI_INCLUDE_DIRS})

# Add ImGui source files directly to target_sources
set(IMGUI_SRC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_sources(ruckig_sim PRIVATE ${IMGUI_SRC})

# Link libraries
 target_link_libraries(ruckig_sim PRIVATE glfw spdlog::spdlog ruckig)
if (WIN32)
    target_link_libraries(ruckig_sim PRIVATE opengl32)
elseif(APPLE)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(OpenGL_LIBRARY OpenGL)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(CoreVideo_LIBRARY CoreVideo)
    target_link_libraries(ruckig_sim PRIVATE ${COCOA_LIBRARY} ${OpenGL_LIBRARY} ${IOKIT_LIBRARY} ${CoreVideo_LIBRARY})
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(ruckig_sim PRIVATE OpenGL::GL)
endif()

# Copy font files to output directory after build
add_custom_command(TARGET ruckig_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/fonts"
        $<TARGET_FILE_DIR:ruckig_sim>/fonts
    COMMENT "Copying all font files to output directory"
)
