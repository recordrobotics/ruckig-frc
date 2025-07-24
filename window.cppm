module;

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <algorithm>

export module window;

export class Window
{
public:
    Window(float size_ratio = 0.7f, int min_width = 640, int min_height = 480, std::string_view title = "Ruckig GLFW Window")
    {
        if (!glfwInit())
        {
            spdlog::error("Failed to initialize GLFW");
            throw std::runtime_error("GLFW init failed");
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
        GLFWmonitor *primary = glfwGetPrimaryMonitor();
        const GLFWvidmode *mode = glfwGetVideoMode(primary);
        if (!primary || !mode)
        {
            spdlog::error("Failed to get primary monitor or video mode");
            glfwTerminate();
            throw std::runtime_error("Monitor or video mode failed");
        }
        int win_width = std::max(static_cast<int>(mode->width * size_ratio), min_width);
        int win_height = std::max(static_cast<int>(mode->height * size_ratio), min_height);
        window_ = glfwCreateWindow(win_width, win_height, title.data(), nullptr, nullptr);
        if (!window_)
        {
            spdlog::error("Failed to create GLFW window");
            glfwTerminate();
            throw std::runtime_error("Window creation failed");
        }
        int xpos = mode->width / 2 - win_width / 2;
        int ypos = mode->height / 2 - win_height / 2;
        glfwSetWindowPos(window_, xpos, ypos);
        spdlog::info("GLFW window created successfully at size {}x{} and centered", win_width, win_height);
        glfwMakeContextCurrent(window_);
    }
    ~Window()
    {
        if (window_)
        {
            glfwDestroyWindow(window_);
            glfwTerminate();
            spdlog::info("GLFW terminated and window destroyed");
        }
    }
    void mainLoop()
    {
        while (!glfwWindowShouldClose(window_))
        {
            glfwPollEvents();
            glfwSwapBuffers(window_);
        }
    }

private:
    GLFWwindow *window_ = nullptr;
};
