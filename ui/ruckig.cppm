module;

#include <spdlog/spdlog.h>
#include <imgui.h>
#include <ruckig/ruckig.hpp>
#include <math.h>
#include <algorithm>
#include <cfloat>
#include <memory>

export module ui.ruckig;

import ui.sources.ruckigsource;
import ui.sources.ruckig_local;
import ui.sources.ruckig_remote;
import ui.uimodule;
import window;

using namespace ruckig;

const int max_iterations = 10000;     // Maximum iterations for trajectory calculation
const size_t max_trail_points = 5000; // Maximum number of trail points

export namespace ui
{
    enum class TelemetrySource
    {
        Local,
        Remote
    };

    class RuckigModule : public UIModule
    {
    public:
        RuckigModule(Window &win) : field_texture(win.loadTexture("assets/field25-annotated.png")),
                                    win(win),
                                    gui_ruckig(0.02),
                                    source(std::make_unique<sources::RuckigLocal>())
        {
            spdlog::info("Ruckig Module initialized with {} DoFs and {} delta time", gui_ruckig.degrees_of_freedom, gui_ruckig.delta_time);

            initSource();
        }

        void render() override
        {
            ImGui::BeginMainMenuBar();

            if (ImGui::BeginMenu("Ruckig"))
            {
                if (ImGui::BeginMenu("Source"))
                {
                    if (ImGui::MenuItem("Local", nullptr, dynamic_cast<sources::RuckigLocal *>(source.get()) != nullptr))
                    {
                        if (source.get())
                        {
                            source->deinit();
                        }

                        source = std::make_unique<sources::RuckigLocal>();
                        initSource();
                        spdlog::info("Switched to Local Ruckig Source");
                    }
                    if (ImGui::MenuItem("Remote", nullptr, dynamic_cast<sources::RuckigRemote *>(source.get()) != nullptr))
                    {
                        if (source.get())
                        {
                            source->deinit();
                        }

                        source = std::make_unique<sources::RuckigRemote>();
                        initSource();
                        spdlog::info("Switched to Remote Ruckig Source");
                    }
                    ImGui::EndMenu();
                }

                ImGui::MenuItem("Show Settings", nullptr, &show_settings);
                ImGui::MenuItem("Show Trajectory", nullptr, &show_trajectory);
                ImGui::MenuItem("Show Waypoints", nullptr, &show_waypointlist);

                if (ImGui::MenuItem("Reset"))
                {
                    source->fullReset();
                    gui_ruckig.reset();
                    gui_ruckig.delta_time = source->getDeltaTime();

                    spdlog::info("Ruckig reset to initial state");
                }

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Field"))
            {
                ImGui::MenuItem("Show Field", nullptr, &show_field);
                ImGui::MenuItem("Show Field Info", nullptr, &show_field_info);

                if (ImGui::MenuItem("Reset Robot"))
                {
                    reset();
                    spdlog::info("Robot reset to initial position");
                }

                ImGui::EndMenu();
            }

            source->drawMenuBar();

            ImGui::EndMainMenuBar();

            if (show_settings)
            {
                ImGui::SetNextWindowPos(ImVec2(0, 40), ImGuiCond_FirstUseEver);
                if (ImGui::Begin("Settings", &show_settings))
                {
                    drawSettings();
                }
                ImGui::End();
            }

            source->update();

            if (show_trajectory)
            {
                ImGui::SetNextWindowPos(ImVec2(320, 40), ImGuiCond_FirstUseEver);
                if (ImGui::Begin("Trajectory", &show_trajectory))
                {
                    drawTrajectory();
                }
                ImGui::End();
            }

            if (show_field)
            {
                ImGui::SetNextWindowPos(ImVec2(800, 40), ImGuiCond_FirstUseEver);
                ImGui::SetNextWindowSize(ImVec2(400, 400), ImGuiCond_FirstUseEver);
                if (ImGui::Begin("Field", &show_field))
                {
                    drawField();
                }
                ImGui::End();
            }

            if (show_field_info)
            {
                ImGui::SetNextWindowPos(ImVec2(800, 600), ImGuiCond_FirstUseEver);
                if (ImGui::Begin("Field Info", &show_field_info))
                {
                    drawFieldInfo();
                }
                ImGui::End();
            }

            if (show_waypointlist)
            {
                ImGui::SetNextWindowPos(ImVec2(600, 600), ImGuiCond_FirstUseEver);
                if (ImGui::Begin("Waypoints", &show_waypointlist))
                {
                    drawWaypointList();
                }
                ImGui::End();
            }

            if (source->getAndClearTitleDirty())
            {
                win.updateTitle(source->getTitle());
            }
        }

    private:
        std::unique_ptr<sources::RuckigSource> source;
        Window &win;

        ImTextureID field_texture;

        Ruckig<3>
            gui_ruckig;

        bool show_settings = true;
        bool show_trajectory = true;
        bool show_field = true;
        bool show_field_info = true;
        bool show_waypointlist = true;

        std::vector<std::pair<float, float>> trail_points{max_trail_points};
        std::vector<std::pair<float, float>> trail_points_old{max_trail_points};
        std::vector<std::pair<float, float>> trail_points_old_old{max_trail_points};
        std::vector<std::pair<float, float>> trail_points_future;

        std::vector<float> robotErrorP, robotErrorV, robotErrorA;
        std::vector<float> targetX, targetY, targetR, currentX, currentY, currentR;

        double field_width_meters = 17.5482504; // meters
        double field_height_meters = 8.0519016; // meters

        void initSource()
        {
            source->updateCallback = std::function<void()>([this]()
                                                           { updateCallback(); });
            source->resetRobotTrigger = std::function<void()>([this]()
                                                              { reset(); });
            source->init();
            source->reset();

            win.updateTitle(source->getTitle());
        }

        void reset()
        {
            source->reset();

            trail_points_old_old = std::move(trail_points_old);
            trail_points_old = std::move(trail_points);
            trail_points.clear();

            robotErrorP.clear();
            robotErrorV.clear();
            robotErrorA.clear();

            targetX.clear();
            targetY.clear();
            targetR.clear();
            currentX.clear();
            currentY.clear();
            currentR.clear();
        }

        void drawSettings()
        {
            source->drawGUI();

            if (gui_ruckig.delta_time != source->getDeltaTime())
            {
                gui_ruckig.delta_time = source->getDeltaTime();
            }
        }

        void updateCallback()
        {
            std::array<double, 3> pos = source->getSetpointPosition();
            double x = pos[0];
            double y = pos[1];
            double r = pos[2];

            double cx, cy, cr, cvx, cvy, cvr, cax, cay, car;

            std::array<double, 3> robotPos = source->getActualPosition();
            cx = robotPos[0];
            cy = robotPos[1];
            cr = robotPos[2];
            std::array<double, 3> robotVel = source->getActualVelocity();
            cvx = robotVel[0];
            cvy = robotVel[1];
            cvr = robotVel[2];
            std::array<double, 3> robotAcc = source->getActualAcceleration();
            cax = robotAcc[0];
            cay = robotAcc[1];
            car = robotAcc[2];

            double ex = x - cx;
            double ey = y - cy;
            double er = r - cr;

            std::array<double, 3> vel = source->getSetpointVelocity();
            std::array<double, 3> acc = source->getSetpointAcceleration();

            double evx = vel[0] - cvx;
            double evy = vel[1] - cvy;
            double evr = vel[2] - cvr;
            double eax = acc[0] - cax;
            double eay = acc[1] - cay;
            double ear = acc[2] - car;

            robotErrorP.push_back(static_cast<float>(std::max(std::abs(ex), std::max(std::abs(ey), std::abs(er)))));
            robotErrorV.push_back(static_cast<float>(std::max(std::abs(evx), std::max(std::abs(evy), std::abs(evr)))));
            robotErrorA.push_back(static_cast<float>(std::max(std::abs(eax), std::max(std::abs(eay), std::abs(ear)))));

            targetX.push_back(static_cast<float>(x));
            targetY.push_back(static_cast<float>(y));
            targetR.push_back(static_cast<float>(r));
            currentX.push_back(static_cast<float>(cx));
            currentY.push_back(static_cast<float>(cy));
            currentR.push_back(static_cast<float>(cr));

            if (trail_points.empty() ||
                std::hypot(static_cast<float>(x) - trail_points.back().first, static_cast<float>(y) - trail_points.back().second) > 0.01f)
            {
                if (trail_points.size() < max_trail_points)
                {
                    trail_points.emplace_back(static_cast<float>(x), static_cast<float>(y));
                }
                else
                {
                    trail_points.erase(trail_points.begin());
                    trail_points.emplace_back(static_cast<float>(x), static_cast<float>(y));
                }
            }
        }

        template <size_t N>
        static void set_minmax(const std::array<const std::vector<float>, N> v, float &mn, float &mx)
        {
            // Compute min/max across all elements of all vectors in v
            bool found = false;
            float local_min = FLT_MAX;
            float local_max = -FLT_MAX;

            for (const auto &vec : v)
            {
                if (!vec.empty())
                {
                    auto [it_min, it_max] = std::minmax_element(vec.begin(), vec.end());
                    if (!found)
                    {
                        local_min = *it_min;
                        local_max = *it_max;
                        found = true;
                    }
                    else
                    {
                        if (*it_min < local_min)
                            local_min = *it_min;
                        if (*it_max > local_max)
                            local_max = *it_max;
                    }
                }
            }

            if (found)
            {
                mn = local_min;
                mx = local_max;
                if (mx - mn <= 1e-6f)
                {
                    mx = mn + 1.0f;
                }
            }
        };

        template <size_t N, typename... Series>
        static void drawMultiLine(const char *label,
                                  const std::array<float, N> &mins,
                                  const std::array<float, N> &maxs,
                                  const Series &...series)
        {
            static_assert(sizeof...(Series) == N, "Number of min/max entries must match number of series");
            static_assert(((std::is_same_v<Series, std::tuple<const std::vector<float> &, ImU32>>) && ...),
                          "Each series must be tuple<const std::vector<float>&, ImU32>");

            const float graph_height = 420.0f;
            const float graph_width = ImGui::GetContentRegionAvail().x;

            // Determine the number of points to draw (minimum length among all series)
            std::array<size_t, N> sizes = {std::get<0>(series).size()...};
            size_t nElements = *std::min_element(sizes.begin(), sizes.end());

            ImGui::Text("%s", label);

            ImVec2 graphPos = ImGui::GetCursorScreenPos();
            ImDrawList *draw_list = ImGui::GetWindowDrawList();
            ImVec2 size = ImVec2(graph_width, graph_height);
            draw_list->AddRect(graphPos, ImVec2(graphPos.x + size.x, graphPos.y + size.y), IM_COL32(200, 200, 200, 255));

            if (nElements >= 2)
            {
                auto plot_line = [&](const std::vector<float> &data, float dMin, float dMax, ImU32 color)
                {
                    float denom = (dMax - dMin);
                    if (denom <= 1e-6f)
                        denom = 1.0f;

                    for (size_t i = 1; i < nElements; ++i)
                    {
                        float x0 = graphPos.x + static_cast<float>(i - 1) * size.x / static_cast<float>(nElements - 1);
                        float x1 = graphPos.x + static_cast<float>(i) * size.x / static_cast<float>(nElements - 1);
                        float y0 = graphPos.y + size.y - ((data[i - 1] - dMin) / denom) * size.y;
                        float y1 = graphPos.y + size.y - ((data[i] - dMin) / denom) * size.y;
                        draw_list->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), color, 2.0f);
                    }
                };

                // Expand the series pack, pairing each with its corresponding min/max
                size_t idx = 0;
                (void)std::initializer_list<int>{
                    (plot_line(std::get<0>(series), mins[idx], maxs[idx], std::get<1>(series)), ++idx, 0)...};
            }

            ImGui::Dummy(size);
        }

        void drawTrajectory()
        {
            InputParameter<3> temp_input;

            temp_input.current_position = source->getSetpointPosition();
            temp_input.current_velocity = source->getSetpointVelocity();
            temp_input.current_acceleration = source->getSetpointAcceleration();
            temp_input.target_position = source->getTargetPosition();
            temp_input.target_velocity = source->getTargetVelocity();
            temp_input.target_acceleration = source->getTargetAcceleration();
            temp_input.max_velocity = source->getMaxVelocity();
            temp_input.max_acceleration = source->getMaxAcceleration();
            temp_input.max_jerk = source->getMaxJerk();

            temp_input.synchronization = source->getSynchronization();
            temp_input.per_dof_synchronization = source->getPerDoFSynchronization();
            temp_input.duration_discretization = source->getDurationDiscretization();

            OutputParameter<3> temp_output;

            std::vector<float> times, p0, v0, a0, p1, v1, a1, p2, v2, a2;
            trail_points_future.clear();
            float global_min = FLT_MAX, global_max = FLT_MIN;

            int iterations = 0;
            gui_ruckig.reset();
            while (gui_ruckig.update(temp_input, temp_output) == Result::Working)
            {
                if (++iterations > max_iterations)
                {
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Error: Trajectory calculation exceeded maximum iterations (%d).", max_iterations);
                    ImGui::PopTextWrapPos();
                    break;
                }

                times.push_back(static_cast<float>(temp_output.time));

                float new_pos_0 = static_cast<float>(temp_output.new_position[0]);
                float new_pos_1 = static_cast<float>(temp_output.new_position[1]);
                float new_pos_2 = static_cast<float>(temp_output.new_position[2]);

                float new_vel_0 = static_cast<float>(temp_output.new_velocity[0]);
                float new_vel_1 = static_cast<float>(temp_output.new_velocity[1]);
                float new_vel_2 = static_cast<float>(temp_output.new_velocity[2]);

                float new_accel_0 = static_cast<float>(temp_output.new_acceleration[0]);
                float new_accel_1 = static_cast<float>(temp_output.new_acceleration[1]);
                float new_accel_2 = static_cast<float>(temp_output.new_acceleration[2]);

                p0.push_back(new_pos_0);
                v0.push_back(new_vel_0);
                a0.push_back(new_accel_0);
                p1.push_back(new_pos_1);
                v1.push_back(new_vel_1);
                a1.push_back(new_accel_1);
                p2.push_back(new_pos_2);
                v2.push_back(new_vel_2);
                a2.push_back(new_accel_2);

                trail_points_future.emplace_back(new_pos_0, new_pos_1);

                global_min = std::min({global_min, new_pos_0, new_pos_1, new_pos_2, new_vel_0, new_vel_1, new_vel_2, new_accel_0, new_accel_1, new_accel_2});
                global_max = std::max({global_max, new_pos_0, new_pos_1, new_pos_2, new_vel_0, new_vel_1, new_vel_2, new_accel_0, new_accel_1, new_accel_2});

                temp_output.pass_to_input(temp_input);
            }

            ImGui::Text("Duration: %.3f s", temp_output.time);
            ImGui::Spacing();

            if (global_max - global_min <= 1e-6f)
            {
                global_max = global_min + 1;
            }

            drawMultiLine("Axis 0 (blue=pos, orange=vel, green=acc)", std::array<float, 3>{global_min, global_min, global_min}, {global_max, global_max, global_max}, std::tuple<const std::vector<float> &, ImU32>{p0, IM_COL32(43, 60, 240, 255)}, std::tuple<const std::vector<float> &, ImU32>{v0, IM_COL32(255, 102, 0, 255)}, std::tuple<const std::vector<float> &, ImU32>{a0, IM_COL32(42, 181, 0, 255)});
            drawMultiLine("Axis 1 (blue=pos, orange=vel, green=acc)", std::array<float, 3>{global_min, global_min, global_min}, {global_max, global_max, global_max}, std::tuple<const std::vector<float> &, ImU32>{p1, IM_COL32(43, 60, 240, 255)}, std::tuple<const std::vector<float> &, ImU32>{v1, IM_COL32(255, 102, 0, 255)}, std::tuple<const std::vector<float> &, ImU32>{a1, IM_COL32(42, 181, 0, 255)});
            drawMultiLine("Axis 2 (blue=pos, orange=vel, green=acc)", std::array<float, 3>{global_min, global_min, global_min}, {global_max, global_max, global_max}, std::tuple<const std::vector<float> &, ImU32>{p2, IM_COL32(43, 60, 240, 255)}, std::tuple<const std::vector<float> &, ImU32>{v2, IM_COL32(255, 102, 0, 255)}, std::tuple<const std::vector<float> &, ImU32>{a2, IM_COL32(42, 181, 0, 255)});

            std::array<float, 3> err_min = {0.0f, 0.0f, 0.0f};
            std::array<float, 3> err_max = {1.0f, 1.0f, 1.0f};

            set_minmax<2>({currentX, targetX}, err_min[0], err_max[0]);
            set_minmax<2>({currentY, targetY}, err_min[1], err_max[1]);
            set_minmax<2>({currentR, targetR}, err_min[2], err_max[2]);

            drawMultiLine("Current vs Target (blue=x, orange=y, green=rot)\nsolid=current, faded=target",
                          std::array<float, 6>{err_min[0], err_min[1], err_min[2], err_min[0], err_min[1], err_min[2]},
                          std::array<float, 6>{err_max[0], err_max[1], err_max[2], err_max[0], err_max[1], err_max[2]},
                          std::tuple<const std::vector<float> &, ImU32>{currentX, IM_COL32(43, 60, 240, 255)},
                          std::tuple<const std::vector<float> &, ImU32>{currentY, IM_COL32(255, 102, 0, 255)},
                          std::tuple<const std::vector<float> &, ImU32>{currentR, IM_COL32(42, 181, 0, 255)},
                          std::tuple<const std::vector<float> &, ImU32>{targetX, IM_COL32(43 / 2, 60 / 2, 240 / 2, 255)},
                          std::tuple<const std::vector<float> &, ImU32>{targetY, IM_COL32(255 / 2, 102 / 2, 0 / 2, 255)},
                          std::tuple<const std::vector<float> &, ImU32>{targetR, IM_COL32(42 / 2, 181 / 2, 0 / 2, 255)});

            err_max = {1.0f, 1.0f, 1.0f};

            set_minmax<1>({robotErrorP}, err_min[0], err_max[0]);
            set_minmax<1>({robotErrorV}, err_min[1], err_max[1]);
            set_minmax<1>({robotErrorA}, err_min[2], err_max[2]);

            drawMultiLine("Robot Error (blue=pos, orange=vel, green=acc)", {0, 0, 0}, err_max,
                          std::tuple<const std::vector<float> &, ImU32>{robotErrorP, IM_COL32(43, 60, 240, 255)}, std::tuple<const std::vector<float> &, ImU32>{robotErrorV, IM_COL32(255, 102, 0, 255)}, std::tuple<const std::vector<float> &, ImU32>{robotErrorA, IM_COL32(42, 181, 0, 255)});
        }

        void drawField()
        {
            // Local static fields for pan and zoom state
            static float zoom_level = 1.0f;
            static ImVec2 pan_offset = {0.0f, 0.0f};
            static bool is_dragging = false;
            static ImVec2 drag_start_mouse;
            static ImVec2 drag_start_pan;

            // Define field bounds
            const float field_min_x = 0, field_max_x = field_width_meters;
            const float field_min_y = 0, field_max_y = field_height_meters;

            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size = ImGui::GetContentRegionAvail();
            if (canvas_size.x < 50.0f)
                canvas_size.x = 50.0f;
            if (canvas_size.y < 50.0f)
                canvas_size.y = 50.0f;

            // Aspect ratio correction
            float field_width = field_max_x - field_min_x;
            float field_height = field_max_y - field_min_y;
            float aspect_field = field_width / field_height;
            float aspect_canvas = canvas_size.x / canvas_size.y;
            ImVec2 draw_size = canvas_size;
            if (aspect_canvas > aspect_field)
            {
                draw_size.x = draw_size.y * aspect_field;
            }
            else
            {
                draw_size.y = draw_size.x / aspect_field;
            }

            // Apply zoom to draw size
            ImVec2 zoomed_draw_size = {draw_size.x * zoom_level, draw_size.y * zoom_level};

            ImDrawList *draw_list = ImGui::GetWindowDrawList();
            ImVec2 draw_offset = canvas_pos;
            // Center the field in the canvas and apply pan offset
            draw_offset.x += (canvas_size.x - zoomed_draw_size.x) * 0.5f + pan_offset.x;
            draw_offset.y += (canvas_size.y - zoomed_draw_size.y) * 0.5f + pan_offset.y;

            // Handle mouse interactions
            ImGui::InvisibleButton("field_canvas", canvas_size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
            bool hovered = ImGui::IsItemHovered();
            ImGuiIO &io = ImGui::GetIO();

            // Handle zoom with scroll wheel
            if (hovered && io.MouseWheel != 0.0f)
            {
                float zoom_factor = 1.0f + io.MouseWheel * 0.1f;
                ImVec2 mouse_pos = io.MousePos;

                // Current centered offset (without pan)
                ImVec2 centered_offset = {
                    canvas_pos.x + (canvas_size.x - zoomed_draw_size.x) * 0.5f,
                    canvas_pos.y + (canvas_size.y - zoomed_draw_size.y) * 0.5f};

                // --- Convert screen (mouse) -> field coordinates BEFORE zoom ---
                float mouse_field_x =
                    ((mouse_pos.x - (centered_offset.x + pan_offset.x)) / zoomed_draw_size.x) * field_width + field_min_x;

                float mouse_field_y =
                    field_min_y + (1.0f - (mouse_pos.y - (centered_offset.y + pan_offset.y)) / zoomed_draw_size.y) * field_height;

                // --- Update zoom ---
                zoom_level = std::clamp(zoom_level * zoom_factor, 0.1f, 10.0f);

                // --- Compute new zoomed size ---
                ImVec2 new_zoomed_size = {draw_size.x * zoom_level, draw_size.y * zoom_level};

                // --- Normalize mouse field position ---
                float target_norm_x = (mouse_field_x - field_min_x) / field_width;
                float target_norm_y = (mouse_field_y - field_min_y) / field_height;

                // --- Recompute centered offset for new zoom ---
                ImVec2 new_centered_offset = {
                    canvas_pos.x + (canvas_size.x - new_zoomed_size.x) * 0.5f,
                    canvas_pos.y + (canvas_size.y - new_zoomed_size.y) * 0.5f};

                // --- Convert field -> screen with new zoom ---
                float expected_screen_x = new_centered_offset.x + pan_offset.x + target_norm_x * new_zoomed_size.x;
                float expected_screen_y = new_centered_offset.y + pan_offset.y + (1.0f - target_norm_y) * new_zoomed_size.y;

                // --- Adjust pan (still measured from center) ---
                pan_offset.x += mouse_pos.x - expected_screen_x;
                pan_offset.y += mouse_pos.y - expected_screen_y;

                // --- Apply final draw offset ---
                zoomed_draw_size = new_zoomed_size;
                draw_offset.x = new_centered_offset.x + pan_offset.x;
                draw_offset.y = new_centered_offset.y + pan_offset.y;
            }

            // Handle panning with left mouse drag
            if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                is_dragging = true;
                drag_start_mouse = io.MousePos;
                drag_start_pan = pan_offset;
            }

            if (is_dragging)
            {
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                {
                    ImVec2 mouse_delta = {io.MousePos.x - drag_start_mouse.x, io.MousePos.y - drag_start_mouse.y};
                    pan_offset = {drag_start_pan.x + mouse_delta.x, drag_start_pan.y + mouse_delta.y};

                    // Update draw offset for current frame
                    draw_offset.x = canvas_pos.x + (canvas_size.x - zoomed_draw_size.x) * 0.5f + pan_offset.x;
                    draw_offset.y = canvas_pos.y + (canvas_size.y - zoomed_draw_size.y) * 0.5f + pan_offset.y;
                }
                else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                {
                    is_dragging = false;
                }
            }

            // Handle right-click to set target position (only if not dragging)
            bool right_clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !is_dragging;
            if (right_clicked)
            {
                ImVec2 mouse_pos = io.MousePos;
                // Convert mouse_pos to field coordinates
                float px = mouse_pos.x - draw_offset.x;
                float py = mouse_pos.y - draw_offset.y;
                // Clamp to zoomed_draw_size
                px = std::clamp(px, 0.0f, zoomed_draw_size.x);
                py = std::clamp(py, 0.0f, zoomed_draw_size.y);
                // Convert to normalized [0,1]
                float norm_x = px / zoomed_draw_size.x;
                float norm_y = 1.0f - (py / zoomed_draw_size.y); // y axis flipped
                // Convert to field coordinates
                float field_x = field_min_x + norm_x * field_width;
                float field_y = field_min_y + norm_y * field_height;
                // Clamp to field bounds
                field_x = std::clamp(field_x, field_min_x, field_max_x);
                field_y = std::clamp(field_y, field_min_y, field_max_y);

                source->updateWaypoints(field_x, field_y);

                robotErrorP.clear();
                robotErrorV.clear();
                robotErrorA.clear();
                targetX.clear();
                targetY.clear();
                targetR.clear();
                currentX.clear();
                currentY.clear();
                currentR.clear();
            }

            // Draw field background image if available
            if (field_texture)
            {
                draw_list->AddImage(field_texture, draw_offset,
                                    ImVec2(draw_offset.x + zoomed_draw_size.x, draw_offset.y + zoomed_draw_size.y));
            }

            // Draw field border
            draw_list->AddRect(draw_offset, ImVec2(draw_offset.x + zoomed_draw_size.x, draw_offset.y + zoomed_draw_size.y), IM_COL32(200, 200, 200, 255), 0.0f, 0, 2.0f);

            auto drawRobot = [&](ImDrawList *draw_list, float rx, float ry, float rot, ImColor color)
            {
                // Draw rotated square (robot)
                float half_size = static_cast<float>(source->getRobotSize() / 2.0); // meters, half side length
                ImVec2 corners[4];                                                  // 4 corners of the square in meters
                for (int i = 0; i < 4; ++i)
                {
                    float angle = rot + M_PI_4 + i * M_PI_2;
                    corners[i].x = rx + half_size * cosf(angle);
                    corners[i].y = ry + half_size * sinf(angle);
                }

                // Convert corners to screen coordinates
                for (int i = 0; i < 4; ++i)
                {
                    float corner_norm_x = (corners[i].x - field_min_x) / field_width;
                    float corner_norm_y = (corners[i].y - field_min_y) / field_height;
                    corners[i].x = draw_offset.x + corner_norm_x * zoomed_draw_size.x;
                    corners[i].y = draw_offset.y + (1.0f - corner_norm_y) * zoomed_draw_size.y; // y axis flipped
                }

                draw_list->AddConvexPolyFilled(corners, 4, IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 180));
                draw_list->AddPolyline(corners, 4, IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 255), true, 2.0f);

                // heading arrow

                float heading_len = half_size * 1.2f;
                ImVec2 heading_start = ImVec2(rx, ry);
                ImVec2 heading_end = ImVec2(rx + heading_len * cosf(rot), ry + heading_len * sinf(rot));

                // Convert to screen coordinates
                float heading_start_norm_x = (heading_start.x - field_min_x) / field_width;
                float heading_start_norm_y = (heading_start.y - field_min_y) / field_height;
                heading_start.x = draw_offset.x + heading_start_norm_x * zoomed_draw_size.x;
                heading_start.y = draw_offset.y + (1.0f - heading_start_norm_y) * zoomed_draw_size.y; // y axis flipped
                float heading_end_norm_x = (heading_end.x - field_min_x) / field_width;
                float heading_end_norm_y = (heading_end.y - field_min_y) / field_height;
                heading_end.x = draw_offset.x + heading_end_norm_x * zoomed_draw_size.x;
                heading_end.y = draw_offset.y + (1.0f - heading_end_norm_y) * zoomed_draw_size.y; // y axis flipped

                draw_list->AddLine(heading_start, heading_end, IM_COL32(102, 102, 255, 255), 2.5f);
            };

            auto drawArrow = [&](ImDrawList *draw_list, float x, float y, float dx, float dy, ImColor color)
            {
                // Convert field coordinates to screen coordinates for start point
                float norm_x = (x - field_min_x) / field_width;
                float norm_y = (y - field_min_y) / field_height;
                float px = draw_offset.x + norm_x * zoomed_draw_size.x;
                float py = draw_offset.y + (1.0f - norm_y) * zoomed_draw_size.y; // y axis flipped

                // Convert field coordinates to screen coordinates for end point
                float end_x = x + dx;
                float end_y = y + dy;
                float end_norm_x = (end_x - field_min_x) / field_width;
                float end_norm_y = (end_y - field_min_y) / field_height;
                float end_px = draw_offset.x + end_norm_x * zoomed_draw_size.x;
                float end_py = draw_offset.y + (1.0f - end_norm_y) * zoomed_draw_size.y; // y axis flipped

                // Draw arrow line
                draw_list->AddLine(ImVec2(px, py), ImVec2(end_px, end_py), IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 255), 2.0f);

                // Calculate arrow head
                float arrow_length = sqrtf((end_px - px) * (end_px - px) + (end_py - py) * (end_py - py));
                if (arrow_length > 10.0f) // Only draw arrow head if line is long enough
                {
                    float arrow_head_size = 8.0f * zoom_level; // Scale arrow head with zoom
                    float angle = atan2f(end_py - py, end_px - px);

                    // Arrow head points
                    ImVec2 head_p1 = ImVec2(
                        end_px - arrow_head_size * cosf(angle - M_PI / 6),
                        end_py - arrow_head_size * sinf(angle - M_PI / 6));
                    ImVec2 head_p2 = ImVec2(
                        end_px - arrow_head_size * cosf(angle + M_PI / 6),
                        end_py - arrow_head_size * sinf(angle + M_PI / 6));

                    // Draw arrow head
                    draw_list->AddLine(ImVec2(end_px, end_py), head_p1, IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 255), 2.0f);
                    draw_list->AddLine(ImVec2(end_px, end_py), head_p2, IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 255), 2.0f);
                }
            };

            // Get robot position and rotation
            std::array<double, 3> pos = source->getActualPosition();
            drawRobot(draw_list, static_cast<float>(pos[0]), static_cast<float>(pos[1]), static_cast<float>(pos[2]), ImColor(43, 60, 240));

            for (auto &wp : source->getWaypoints())
            {
                drawRobot(draw_list, static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.r), wp.color);
                drawArrow(draw_list, static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.vx), static_cast<float>(wp.vy), wp.color);
                drawArrow(draw_list, static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.ax), static_cast<float>(wp.ay), wp.color);
            }

            std::array<double, 3> setpointPos = source->getSetpointPosition();
            drawRobot(draw_list, static_cast<float>(setpointPos[0]), static_cast<float>(setpointPos[1]), static_cast<float>(setpointPos[2]), ImColor(240, 60, 43));

            // Draw trail
            auto drawTrail = [&](std::vector<std::pair<float, float>> &trail_points, ImU8 alpha)
            {
                if (trail_points.size() > 1)
                {
                    for (size_t i = 1; i < trail_points.size(); ++i)
                    {
                        float rx0 = trail_points[i - 1].first;
                        float ry0 = trail_points[i - 1].second;
                        float rx1 = trail_points[i].first;
                        float ry1 = trail_points[i].second;

                        // Map to field coordinates
                        float norm_x0 = (rx0 - field_min_x) / field_width;
                        float norm_y0 = (ry0 - field_min_y) / field_height;
                        float px0 = draw_offset.x + norm_x0 * zoomed_draw_size.x;
                        float py0 = draw_offset.y + (1.0f - norm_y0) * zoomed_draw_size.y;

                        float norm_x1 = (rx1 - field_min_x) / field_width;
                        float norm_y1 = (ry1 - field_min_y) / field_height;
                        float px1 = draw_offset.x + norm_x1 * zoomed_draw_size.x;
                        float py1 = draw_offset.y + (1.0f - norm_y1) * zoomed_draw_size.y;

                        draw_list->AddLine(ImVec2(px0, py0), ImVec2(px1, py1), IM_COL32(255, 255, 255, alpha), 2.0f);
                    }
                }
            };

            drawTrail(trail_points, 180);        // Draw current trail with alpha 180
            drawTrail(trail_points_old, 120);    // Draw old trail with alpha 120
            drawTrail(trail_points_old_old, 60); // Draw older trail with alpha 60
            drawTrail(trail_points_future, 30);  // Draw future trail with alpha 30
        }

        inline std::string_view to_string(Result r)
        {
            switch (r)
            {
            case Result::Working:
                return "Working";
            case Result::Finished:
                return "Finished";
            case Result::Error:
                return "Error";
            case Result::ErrorInvalidInput:
                return "Error: Invalid Input";
            case Result::ErrorTrajectoryDuration:
                return "Error: Trajectory Duration";
            case Result::ErrorPositionalLimits:
                return "Error: Positional Limits";
            case Result::ErrorZeroLimits:
                return "Error: Zero Limits";
            case Result::ErrorExecutionTimeCalculation:
                return "Error: Execution Time Calculation";
            case Result::ErrorSynchronizationCalculation:
                return "Error: Synchronization Calculation";
            default:
                return "Unknown Result";
            }
        }

        void drawFieldInfo()
        {
            ImGui::Text("Field Size: %.1f x %.1f", field_width_meters, field_height_meters);
            ImGui::Text("Robot Size: %.2f m", source->getRobotSize());

            std::array<double, 3> actualPosition = source->getActualPosition();
            std::array<double, 3> actualVelocity = source->getActualVelocity();
            std::array<double, 3> targetPosition = source->getTargetPosition();
            std::array<double, 3> setpointVelocity = source->getSetpointVelocity();
            std::array<double, 3> setpointAcceleration = source->getSetpointAcceleration();
            ImGui::Text("Robot Position: (%.2f, %.2f, %.2f)", actualPosition[0], actualPosition[1], actualPosition[2]);
            ImGui::Text("Robot Velocity: (%.2f, %.2f, %.2f)", actualVelocity[0], actualVelocity[1], actualVelocity[2]);
            ImGui::Text("Target Position: (%.2f, %.2f, %.2f)", targetPosition[0], targetPosition[1], targetPosition[2]);
            ImGui::Text("Setpoint Velocity: (%.2f, %.2f, %.2f)", setpointVelocity[0], setpointVelocity[1], setpointVelocity[2]);
            ImGui::Text("Current Acceleration: (%.2f, %.2f, %.2f)", setpointAcceleration[0], setpointAcceleration[1], setpointAcceleration[2]);
            ImGui::Spacing();

            ImGui::Text("Delta Time: %.3f s", source->getActualDeltaTime());
            ImGui::Text("Calculation Time: %.1f µs", source->getCalculationTime());
            ImGui::Text("Result: %s", to_string(source->getResult()).data());
            ImGui::Spacing();

            if (!source->isMonotonic())
            {
                ImGui::Text("Speed");
                ImGui::SameLine();
                ImGui::SliderInt("##speed", &source->speed, 0, 20, "%dx", ImGuiSliderFlags_AlwaysClamp);
            }
        }

        void drawWaypointList()
        {
            std::vector<sources::Pose2d> waypoints = source->getWaypoints();
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                auto &wp = waypoints[i];
                ImGui::PushID(static_cast<int>(i));

                if (wp.editing)
                {
                    // Show editable input fields when in edit mode
                    ImGui::Text("Waypoint %zu:", i + 1);
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(60);
                    {
                        double min_x = -10.0, max_x = 10.0;
                        ImGui::DragScalar("##x", ImGuiDataType_Double, &wp.x, 0.01f, &min_x, &max_x, "%.2f");
                    }
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(60);
                    {
                        double min_y = -10.0, max_y = 10.0;
                        ImGui::DragScalar("##y", ImGuiDataType_Double, &wp.y, 0.01f, &min_y, &max_y, "%.2f");
                    }
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(60);
                    {
                        double min_r = -3.14159, max_r = 3.14159;
                        ImGui::DragScalar("##r", ImGuiDataType_Double, &wp.r, 0.01f, &min_r, &max_r, "%.2f");
                    }
                }
                else
                {
                    // Display waypoint text normally
                    ImGui::Text("Waypoint %zu: (%.2f, %.2f, %.2f)", i + 1, wp.x, wp.y, wp.r);
                }

                // Add Edit button
                if (source->canEditWaypoints())
                {
                    ImGui::SameLine();
                    if (wp.editing)
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(60, 180, 60, 255));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(80, 200, 80, 255));
                        ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(40, 160, 40, 255));
                        if (ImGui::SmallButton("Save"))
                        {
                            wp.editing = false;
                        }
                        ImGui::PopStyleColor(3);
                    }
                    else
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(60, 120, 220, 255));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(80, 140, 255, 255));
                        ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(40, 100, 180, 255));
                        if (ImGui::SmallButton("Edit"))
                        {
                            wp.editing = true;
                        }
                        ImGui::PopStyleColor(3);
                    }

                    source->editWaypoint(i, wp);
                }

                // Add X button
                if (source->canEditWaypoints())
                {
                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(220, 60, 60, 255));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(255, 80, 80, 255));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(180, 40, 40, 255));
                    if (ImGui::SmallButton("X"))
                    {
                        source->removeWaypoint(i);
                        ImGui::PopStyleColor(3);
                        ImGui::PopID();
                        break; // Exit loop to avoid issues with modified vector
                    }
                    ImGui::PopStyleColor(3);
                }

                ImGui::PopID();
            }
            if (source->canEditWaypoints() && ImGui::Button("Add Waypoint"))
            {
                source->addWaypoint();
            }
        }
    };
}