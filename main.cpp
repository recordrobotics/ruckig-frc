#include <spdlog/spdlog.h>
#include <algorithm>

import window;

int main()
{
    try
    {
        Window win;
        win.mainLoop();
    }
    catch (const std::exception &e)
    {
        spdlog::error("Exception: {}", e.what());
        return -1;
    }
    return 0;
}
