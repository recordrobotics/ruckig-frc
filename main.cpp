#include <spdlog/spdlog.h>
#include <algorithm>

import window;
import ui.ruckig;

int main()
{
    try
    {
        Window win;
        win.addUIModule(new ui::Ruckig());
        win.mainLoop();
    }
    catch (const std::exception &e)
    {
        spdlog::error("Exception: {}", e.what());
        return -1;
    }
    return 0;
}
