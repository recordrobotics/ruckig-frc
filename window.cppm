module;

#include <spdlog/spdlog.h>
#include <algorithm>
#include <imgui.h>
#include <vector>

#include <stb_image.h>

#if BACKEND_OPENGL

#include <GLFW/glfw3.h>
#include <gl/GL.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#elif BACKEND_DX11

#include <windows.h>
#include <wrl/client.h>
#include <d3d11.h>
#include <backends/imgui_impl_dx11.h>
#include <backends/imgui_impl_win32.h>

#elif BACKEND_METAL

#define NS_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_metal.h>

#include "window-metal.h"

#else

#error "Unsupported graphics backend. Please set BACKEND to OpenGL, DX11, or Metal."

#endif

#if BACKEND_DX11
// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
#endif

export module window;

import ui.uimodule;

using namespace ui;

#if BACKEND_DX11
using Microsoft::WRL::ComPtr;
#endif

export class Window
{
public:
    Window(float size_ratio = 0.7f, int min_width = 640, int min_height = 480, std::string_view title = "Ruckig FRC")
    {
#if BACKEND_OPENGL or BACKEND_METAL
        initGLFW(size_ratio, min_width, min_height, title);
#elif BACKEND_DX11
        initDX11(size_ratio, min_width, min_height, title);
#endif

        initImGui();
    }

    ~Window()
    {
#if BACKEND_OPENGL or BACKEND_METAL
        if (window_)
#elif BACKEND_DX11
#endif
        {
            shutdownImGui();
#if BACKEND_OPENGL or BACKEND_METAL
            glfwDestroyWindow(window_);
            glfwTerminate();
            spdlog::info("GLFW terminated and window destroyed");
#elif BACKEND_DX11
            if (hwnd_)
                DestroyWindow(hwnd_);
            UnregisterClass(wc_.lpszClassName, wc_.hInstance);
            spdlog::info("DX11 device and resources released");
#endif
        }
    }

    void mainLoop()
    {
#if BACKEND_OPENGL or BACKEND_METAL
        while (!glfwWindowShouldClose(window_))
        {
            glfwPollEvents();

#if BACKEND_OPENGL
            if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED) != 0)
            {
                ImGui_ImplGlfw_Sleep(10);
                continue;
            }
#endif

            renderImGuiFrame();

#if BACKEND_OPENGL
            glfwSwapBuffers(window_);
#endif
        }
#elif BACKEND_DX11
        bool done = false;
        while (!done)
        {
            MSG msg = {};
            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
                if (msg.message == WM_QUIT)
                    done = true;
            }
            if (done)
                break;

            if (swapchain_occluded && swap_chain_->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED)
            {
                Sleep(10);
                continue;
            }
            swapchain_occluded = false;

            if (resizeWidth != 0 && resizeHeight != 0)
            {
                // Release old render target view before resizing
                render_target_view_.Reset();
                // Resize swap chain
                swap_chain_->ResizeBuffers(0, resizeWidth, resizeHeight, DXGI_FORMAT_UNKNOWN, 0);
                resizeWidth = 0; // Reset after resizing
                resizeHeight = 0;
                createDX11RenderTarget();
            }

            renderImGuiFrame();

            HRESULT hr = swap_chain_->Present(1, 0);
            swapchain_occluded = (hr == DXGI_STATUS_OCCLUDED);
        }
#endif
    }

    void addUIModule(UIModule *module)
    {
        uimodules_.push_back(module);
    }

    void removeUIModule(UIModule *module)
    {
        uimodules_.erase(std::remove(uimodules_.begin(), uimodules_.end(), module), uimodules_.end());
    }

    void updateTitle(std::string_view title)
    {
#if BACKEND_OPENGL or BACKEND_METAL
        if (window_)
        {
            glfwSetWindowTitle(window_, title.data());
        }
#elif BACKEND_DX11
        if (hwnd_)
        {
            std::string title_str(title); // Ensure null-terminated string
            SetWindowText(hwnd_, title_str.c_str());
        }
#endif
    }

    ImTextureID loadTexture(std::string_view path)
    {
#if BACKEND_OPENGL
        // Load image using stb_image or similar
        int width, height, channels;
        unsigned char *data = stbi_load(path.data(), &width, &height, &channels, 4);
        if (!data)
        {
            spdlog::error("Failed to load texture: {}", path);
            return ImTextureID_Invalid;
        }

        GLuint texture_id;
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glBindTexture(GL_TEXTURE_2D, 0);

        stbi_image_free(data);
        return (ImTextureID)(uintptr_t)texture_id;

#elif BACKEND_DX11
        // Load image using stb_image or similar
        int width, height, channels;
        unsigned char *data = stbi_load(path.data(), &width, &height, &channels, 4);
        if (!data)
        {
            spdlog::error("Failed to load texture: {}", path);
            return ImTextureID_Invalid;
        }

        D3D11_TEXTURE2D_DESC desc = {};
        desc.Width = width;
        desc.Height = height;
        desc.MipLevels = 1;
        desc.ArraySize = 1;
        desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        desc.SampleDesc.Count = 1;
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

        D3D11_SUBRESOURCE_DATA subResource = {};
        subResource.pSysMem = data;
        subResource.SysMemPitch = width * 4;

        ComPtr<ID3D11Texture2D> texture;
        HRESULT hr = device_->CreateTexture2D(&desc, &subResource, &texture);
        stbi_image_free(data);

        if (FAILED(hr))
        {
            spdlog::error("Failed to create D3D11 texture: {}", path);
            return ImTextureID_Invalid;
        }

        ComPtr<ID3D11ShaderResourceView> srv;
        hr = device_->CreateShaderResourceView(texture.Get(), nullptr, &srv);
        if (FAILED(hr))
        {
            spdlog::error("Failed to create D3D11 shader resource view: {}", path);
            return ImTextureID_Invalid;
        }

        return (ImTextureID)srv.Detach();

#elif BACKEND_METAL
        // Load image using stb_image or similar
        int width, height, channels;
        unsigned char *data = stbi_load(path.data(), &width, &height, &channels, 4);
        if (!data)
        {
            spdlog::error("Failed to load texture: {}", path);
            return ImTextureID_Invalid;
        }

        MTL::TextureDescriptor *textureDescriptor = MTL::TextureDescriptor::texture2DDescriptor(
            MTL::PixelFormat::PixelFormatRGBA8Unorm, width, height, false);
        textureDescriptor->setUsage(MTL::TextureUsageShaderRead);

        MTL::Texture *texture = device_->newTexture(textureDescriptor);
        if (!texture)
        {
            spdlog::error("Failed to create Metal texture: {}", path);
            stbi_image_free(data);
            return ImTextureID_Invalid;
        }

        MTL::Region region = MTL::Region::Make2D(0, 0, width, height);
        texture->replaceRegion(region, 0, data, width * 4);

        stbi_image_free(data);
        return (ImTextureID)texture;
#endif
    }

private:
#if BACKEND_OPENGL or BACKEND_METAL
    static void glfwErrorCallback(int error, const char *description)
    {
        spdlog::error("GLFW Error [{}]: {}", error, description);
    }
#endif

    inline void initGLFW(float size_ratio, int min_width, int min_height, std::string_view title)
    {
#if BACKEND_OPENGL or BACKEND_METAL
#if BACKEND_OPENGL
        spdlog::info("Initializing GLFW for OpenGL backend");
#elif BACKEND_METAL
        spdlog::info("Initializing GLFW for Metal backend");
#endif

        glfwSetErrorCallback(glfwErrorCallback);
        if (!glfwInit())
        {
            spdlog::error("Failed to initialize GLFW");
            throw std::runtime_error("GLFW init failed");
        }
#if BACKEND_OPENGL
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
#elif BACKEND_METAL
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
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

#if BACKEND_METAL
        device_ = MTL::CreateSystemDefaultDevice();
        commandQueue_ = device_->newCommandQueue();
#endif

        int xpos = mode->width / 2 - win_width / 2;
        int ypos = mode->height / 2 - win_height / 2;
        glfwSetWindowPos(window_, xpos, ypos);
        spdlog::info("GLFW window created successfully with size {}x{} and centered", win_width, win_height);

        glfwMaximizeWindow(window_);
        spdlog::info("GLFW window maximized");

#if BACKEND_OPENGL
        glfwMakeContextCurrent(window_);
        spdlog::info("GLFW context made current");
#endif
#endif
    }

#if BACKEND_DX11

    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
    {
        if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
            return true;

        switch (uMsg)
        {
        case WM_SIZE:
        {
            if (wParam == SIZE_MINIMIZED)
                return 0;

            Window *win = reinterpret_cast<Window *>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
            if (win)
            {
                win->resizeWidth = (UINT)LOWORD(lParam); // Queue resize
                win->resizeHeight = (UINT)HIWORD(lParam);
            }
            return 0;
        }
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
                return 0;
            break;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        }

        return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }
#endif

    inline void initDX11(float size_ratio, int min_width, int min_height, std::string_view title)
    {
#if BACKEND_DX11
        spdlog::info("Initializing DX11 backend");

        ImGui_ImplWin32_EnableDpiAwareness();

        // Create Win32 window
        wc_.lpfnWndProc = WindowProc;
        wc_.hInstance = GetModuleHandle(nullptr);
        wc_.lpszClassName = TEXT("RuckigFrcWindowClass");
        RegisterClass(&wc_);

        // Get primary monitor info using Win32 API
        DEVMODE devMode = {};
        devMode.dmSize = sizeof(DEVMODE);
        if (!EnumDisplaySettings(nullptr, ENUM_CURRENT_SETTINGS, &devMode))
        {
            spdlog::error("Failed to get primary monitor info via Win32 API");
            throw std::runtime_error("Monitor info failed");
        }
        int screen_width = devMode.dmPelsWidth;
        int screen_height = devMode.dmPelsHeight;

        int win_width = max(static_cast<int>(screen_width * size_ratio), min_width);
        int win_height = max(static_cast<int>(screen_height * size_ratio), min_height);
        int xpos = screen_width / 2 - win_width / 2;
        int ypos = screen_height / 2 - win_height / 2;

        std::string title_str(title); // Ensure null-terminated string
        HWND hwnd = CreateWindowEx(0, wc_.lpszClassName, title_str.c_str(), WS_OVERLAPPEDWINDOW,
                                   xpos, ypos, win_width, win_height,
                                   nullptr, nullptr, wc_.hInstance, nullptr);
        if (!hwnd)
        {
            spdlog::error("Failed to create Win32 window for DX11");
            throw std::runtime_error("Win32 window creation failed");
        }

        // Store pointer to Window instance for resize handling
        SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

        spdlog::info("Win32 window created successfully with size {}x{} and centered", win_width, win_height);

        // Create DX11 device and swap chain
        DXGI_SWAP_CHAIN_DESC scd = {};
        ZeroMemory(&scd, sizeof(scd));
        scd.BufferCount = 2;
        scd.BufferDesc.Width = 0;
        scd.BufferDesc.Height = 0;
        scd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        scd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
        scd.OutputWindow = hwnd;
        scd.SampleDesc.Count = 1;
        scd.Windowed = TRUE;
        scd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

        UINT createDeviceFlags = 0;
#if _DEBUG
        createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
        const D3D_FEATURE_LEVEL featureLevelArray[] = {
            D3D_FEATURE_LEVEL_11_1,
            D3D_FEATURE_LEVEL_11_0,
            D3D_FEATURE_LEVEL_10_0,
        };
        // Try to create DX11 device and swap chain with hardware acceleration first
        HRESULT hr = D3D11CreateDeviceAndSwapChain(
            nullptr,
            D3D_DRIVER_TYPE_HARDWARE,
            nullptr,
            createDeviceFlags,
            featureLevelArray,
            static_cast<UINT>(std::size(featureLevelArray)),
            D3D11_SDK_VERSION,
            &scd,
            &swap_chain_,
            &device_,
            &feature_level_,
            &device_context_);
        if (hr == DXGI_ERROR_UNSUPPORTED) // Try high-performance WARP software driver if hardware is not available.
            hr = D3D11CreateDeviceAndSwapChain(
                nullptr,
                D3D_DRIVER_TYPE_WARP,
                nullptr,
                createDeviceFlags,
                featureLevelArray,
                static_cast<UINT>(std::size(featureLevelArray)),
                D3D11_SDK_VERSION,
                &scd,
                &swap_chain_,
                &device_,
                &feature_level_,
                &device_context_);
        if (hr != S_OK)
        {
            spdlog::error("Failed to initialize DX11");
            throw std::runtime_error("DX11 init failed");
        }

        createDX11RenderTarget();
        spdlog::info("DX11 device and swap chain created successfully");
        // Store window handle for ImGui
        hwnd_ = hwnd;

        ShowWindow(hwnd, SW_MAXIMIZE);
        UpdateWindow(hwnd);
#endif
    }

#if BACKEND_DX11
    inline void createDX11RenderTarget()
    {
        // Create render target view
        ComPtr<ID3D11Texture2D> backBuffer;
        HRESULT hr = swap_chain_->GetBuffer(0, IID_PPV_ARGS(&backBuffer));
        if (FAILED(hr))
        {
            spdlog::error("Failed to get DX11 back buffer");
            throw std::runtime_error("DX11 back buffer failed");
        }
        hr = device_->CreateRenderTargetView(backBuffer.Get(), nullptr, render_target_view_.GetAddressOf());
        if (FAILED(hr))
        {
            spdlog::error("Failed to create DX11 render target view");
            throw std::runtime_error("DX11 render target view failed");
        }
    }
#endif

    void initImGui()
    {
        spdlog::info("Initializing ImGui context");
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO &io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        ImGui::StyleColorsDark();

#if BACKEND_OPENGL or BACKEND_METAL
        float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
#elif BACKEND_DX11
        float main_scale = ImGui_ImplWin32_GetDpiScaleForHwnd(hwnd_);
#endif
        // Setup scaling
        ImGuiStyle &style = ImGui::GetStyle();
        style.ScaleAllSizes(main_scale);
        style.FontScaleDpi = main_scale;

        // High DPI font loading
        float font_size = 16.0f;
        io.Fonts->Clear();
        io.Fonts->AddFontFromFileTTF("fonts/RobotoMono-VariableFont_wght.ttf", font_size);

// Set framebuffer scale for ImGui
#if BACKEND_OPENGL or BACKEND_METAL
        int fb_w, fb_h;
        glfwGetFramebufferSize(window_, &fb_w, &fb_h);
        int win_w, win_h;
        glfwGetWindowSize(window_, &win_w, &win_h);
        io.DisplayFramebufferScale = ImVec2((float)fb_w / win_w, (float)fb_h / win_h);
        spdlog::info("ImGui framebuffer scale set to ({}, {})", io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
#endif

#if BACKEND_OPENGL
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init("#version 330");
#elif BACKEND_DX11
        ImGui_ImplWin32_Init(hwnd_);
        ImGui_ImplDX11_Init(device_.Get(), device_context_.Get());
#elif BACKEND_METAL
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplMetal_Init(device_);

        layer_ = CA::MetalLayer::layer();
        layer_->setDevice(device_);
        layer_->setPixelFormat(MTL::PixelFormat::PixelFormatBGRA8Unorm);
        SetMetalLayerForGLFWWindow(layer_, window_);

        renderPassDescriptor_ = MTL::RenderPassDescriptor::renderPassDescriptor();
#endif

        spdlog::info("ImGui initialized successfully");
    }

    void shutdownImGui()
    {
        spdlog::info("Shutting down ImGui");
#if BACKEND_OPENGL
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
#elif BACKEND_DX11
        ImGui_ImplDX11_Shutdown();
        ImGui_ImplWin32_Shutdown();
#elif BACKEND_METAL
        ImGui_ImplMetal_Shutdown();
        ImGui_ImplGlfw_Shutdown();
#endif
        ImGui::DestroyContext();
        spdlog::info("ImGui shutdown complete");
    }

    void renderImGuiFrame()
    {
#if BACKEND_OPENGL or BACKEND_METAL
        int display_w, display_h;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
#if BACKEND_OPENGL
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
#elif BACKEND_METAL
        layer_->setDrawableSize({static_cast<CGFloat>(display_w), static_cast<CGFloat>(display_h)});
        CA::MetalDrawable *drawable = layer_->nextDrawable();

        MTL::CommandBuffer *commandBuffer = commandQueue_->commandBuffer();
        renderPassDescriptor_->colorAttachments()->object(0)->setClearColor(MTL::ClearColor::Make(0.1, 0.1, 0.1, 1.0));
        renderPassDescriptor_->colorAttachments()->object(0)->setTexture(drawable->texture());
        renderPassDescriptor_->colorAttachments()->object(0)->setLoadAction(MTL::LoadAction::LoadActionClear);
        renderPassDescriptor_->colorAttachments()->object(0)->setStoreAction(MTL::StoreAction::StoreActionStore);
        MTL::RenderCommandEncoder *renderEncoder = commandBuffer->renderCommandEncoder(renderPassDescriptor_);
        renderEncoder->pushDebugGroup(NS::String::string("ruckig-frc", NS::StringEncoding::ASCIIStringEncoding));
#endif
#elif BACKEND_DX11
        device_context_->OMSetRenderTargets(1, render_target_view_.GetAddressOf(), nullptr);
        float clear_color[4] = {0.1f, 0.1f, 0.1f, 1.0f};
        device_context_->ClearRenderTargetView(render_target_view_.Get(), clear_color);
#endif

#if BACKEND_OPENGL
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
#elif BACKEND_DX11
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
#elif BACKEND_METAL
        ImGui_ImplMetal_NewFrame(renderPassDescriptor_);
        ImGui_ImplGlfw_NewFrame();
#endif
        ImGui::NewFrame();

        for (auto *module : uimodules_)
        {
            if (module)
                module->render();
        }

        ImGui::Render();

#if BACKEND_OPENGL
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#elif BACKEND_DX11
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
#elif BACKEND_METAL
        ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(), commandBuffer, renderEncoder);
        renderEncoder->popDebugGroup();
        renderEncoder->endEncoding();

        commandBuffer->presentDrawable(drawable);
        commandBuffer->commit();
#endif
    }

#if BACKEND_OPENGL
    GLFWwindow *window_ = nullptr;
#elif BACKEND_DX11
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> device_context_;
    ComPtr<IDXGISwapChain> swap_chain_;
    ComPtr<ID3D11RenderTargetView> render_target_view_;
    D3D_FEATURE_LEVEL feature_level_;
    HWND hwnd_ = nullptr;
    WNDCLASS wc_ = {};
    bool swapchain_occluded = false;
    UINT resizeWidth = 0, resizeHeight = 0;
#elif BACKEND_METAL
    GLFWwindow *window_ = nullptr;
    MTL::Device *device_ = nullptr;
    MTL::CommandQueue *commandQueue_ = nullptr;
    MTL::RenderPassDescriptor *renderPassDescriptor_ = nullptr;
    CA::MetalLayer *layer_;
#endif

    std::vector<UIModule *> uimodules_;
};
