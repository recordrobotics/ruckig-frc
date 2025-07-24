module;

#include <imgui.h>
#include <ruckig/ruckig.hpp>

export module ui.ruckig;

import ui.uimodule;

export namespace ui
{
    export class Ruckig : public UIModule
    {
    public:
        void render() override
        {
            ImGui::Begin("Test Module RUCKIG 2");
            ImGui::Text("This is a test module rendering in ImGui.");
            ImGui::End();
        }
    };
}