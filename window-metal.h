#pragma once

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

    void SetMetalLayerForGLFWWindow(CA::MetalLayer *layer, GLFWwindow *window);

#ifdef __cplusplus
}
#endif
