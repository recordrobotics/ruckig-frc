module;

#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <vector>

export module window;

import ui.uimodule;

using namespace ui;

export class Window
{
public:
    Window(float size_ratio = 0.7f, int min_width = 640, int min_height = 480, std::string_view title = "Ruckig FRC Simulation")
    {
        glfwSetErrorCallback(glfwErrorCallback);
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
        spdlog::info("GLFW window created successfully with size {}x{} and centered", win_width, win_height);

        glfwMaximizeWindow(window_);
        spdlog::info("GLFW window maximized");

        glfwMakeContextCurrent(window_);
        spdlog::info("GLFW context made current");

        initImGui();
    }

    ~Window()
    {
        if (window_)
        {
            shutdownImGui();
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

            if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED) != 0)
            {
                ImGui_ImplGlfw_Sleep(10);
                continue;
            }

            renderImGuiFrame();

            glfwSwapBuffers(window_);
        }
    }

    void addUIModule(UIModule *module)
    {
        uimodules_.push_back(module);
    }

    void removeUIModule(UIModule *module)
    {
        uimodules_.erase(std::remove(uimodules_.begin(), uimodules_.end(), module), uimodules_.end());
    }

private:
    static void glfwErrorCallback(int error, const char *description)
    {
        spdlog::error("GLFW Error [{}]: {}", error, description);
    }

    void initImGui()
    {
        spdlog::info("Initializing ImGui context");
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO &io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        ImGui::StyleColorsDark();

        float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
        // Setup scaling
        ImGuiStyle &style = ImGui::GetStyle();
        style.ScaleAllSizes(main_scale);
        style.FontScaleDpi = main_scale;

        // High DPI font loading
        float font_size = 16.0f;
        io.Fonts->Clear();
        io.Fonts->AddFontFromFileTTF("fonts/RobotoMono-VariableFont_wght.ttf", font_size);

        // Set framebuffer scale for ImGui
        int fb_w, fb_h;
        glfwGetFramebufferSize(window_, &fb_w, &fb_h);
        int win_w, win_h;
        glfwGetWindowSize(window_, &win_w, &win_h);
        io.DisplayFramebufferScale = ImVec2((float)fb_w / win_w, (float)fb_h / win_h);
        spdlog::info("ImGui framebuffer scale set to ({}, {})", io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);

        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init("#version 330");
        spdlog::info("ImGui initialized successfully");
    }

    void shutdownImGui()
    {
        spdlog::info("Shutting down ImGui");
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        spdlog::info("ImGui shutdown complete");
    }

    void renderImGuiFrame()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        for (auto *module : uimodules_)
        {
            if (module)
                module->render();
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    GLFWwindow *window_ = nullptr;
    std::vector<UIModule *> uimodules_;
};
