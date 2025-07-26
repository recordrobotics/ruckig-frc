#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>

extern "C" void SetMetalLayerForGLFWWindow(CAMetalLayer* layer, GLFWwindow* window) {
    NSWindow *nswin = (NSWindow *)glfwGetCocoaWindow(window);
    nswin.contentView.layer = layer;
    nswin.contentView.wantsLayer = YES;
}

