module;

#include <spdlog/spdlog.h>
#include <imgui.h>
#include <ruckig/ruckig.hpp>
#include <math.h>

export module ui.ruckig;

import ui.uimodule;
import robot;
import pid;

using namespace ruckig;

const int max_iterations = 10000;     // Maximum iterations for trajectory calculation
const size_t max_trail_points = 5000; // Maximum number of trail points

export namespace ui
{
    export enum class RobotControlMode {
        PositionHold,
        Velocity
    };

    export class RuckigModule : public UIModule
    {
    public:
        RuckigModule() : ruckig(0.02), gui_ruckig(0.02), robot()
        {
            spdlog::info("Ruckig Module initialized with {} DoFs and {} delta time", ruckig.degrees_of_freedom, ruckig.delta_time);

            init();
            resetRobot();
        }

        void render() override
        {
            ImGui::BeginMainMenuBar();

            if (ImGui::BeginMenu("Ruckig"))
            {
                ImGui::MenuItem("Show Settings", nullptr, &show_settings);
                ImGui::MenuItem("Show Trajectory", nullptr, &show_trajectory);

                if (ImGui::MenuItem("Reset"))
                {
                    ruckig.reset();
                    gui_ruckig.reset();
                    ruckig.delta_time = 0.02; // Reset delta time
                    gui_ruckig.delta_time = ruckig.delta_time;
                    input = InputParameter<3>();
                    output = OutputParameter<3>();
                    init();

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
                    resetRobot();
                    spdlog::info("Robot reset to initial position");
                }

                ImGui::EndMenu();
            }

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

            double current_time = ImGui::GetTime();
            if (ruckig.delta_time > 0.0 && (current_time - last_time) >= ruckig.delta_time)
            {
                last_time = current_time;

                for (int i = 0; i < speed_up; i++)
                {
                    // Calculate trajectory
                    auto result = ruckig.update(input, output);
                    output.pass_to_input(input);

                    switch (control_mode)
                    {
                    case RobotControlMode::PositionHold:
                    {
                        double ix = 0;
                        double iy = 0;
                        double ir = 0;

                        robot.getPosition(ix, iy, ir);

                        double x = output.new_position[0];
                        double y = output.new_position[1];
                        double r = output.new_position[2];
                        double vx = position_pid_x.calculate(x, ix, position_pid_kp, position_pid_ki, position_pid_kd, ruckig.delta_time);
                        double vy = position_pid_y.calculate(y, iy, position_pid_kp, position_pid_ki, position_pid_kd, ruckig.delta_time);
                        double vr = position_pid_r.calculate(r, ir, rotation_pid_kp, rotation_pid_ki, rotation_pid_kd, ruckig.delta_time);
                        double ax = 0;
                        double ay = 0;
                        double ar = 0;
                        robot.update(ruckig.delta_time, x, y, r, vx, vy, vr,
                                     ax, ay, ar);
                    }
                    break;
                    case RobotControlMode::Velocity:
                    {
                        if (result == Result::Working)
                        {
                            double x = output.new_position[0];
                            double y = output.new_position[1];
                            double r = output.new_position[2];
                            double vx = output.new_velocity[0];
                            double vy = output.new_velocity[1];
                            double vr = output.new_velocity[2];
                            robot.update(ruckig.delta_time, x, y, r, vx, vy, vr,
                                         input.current_acceleration[0], input.current_acceleration[1], input.current_acceleration[2]);
                            input.current_position[0] = x;
                            input.current_position[1] = y;
                            input.current_position[2] = r;
                            input.current_velocity[0] = vx;
                            input.current_velocity[1] = vy;
                            input.current_velocity[2] = vr;
                        }
                        else if (result == Result::Finished)
                        {
                            spdlog::info("Trajectory calculation finished successfully");
                            break;
                        }
                        else
                        {
                            spdlog::error("Trajectory calculation failed: {}", std::to_string(result));
                        }
                    }
                    break;
                    }
                }
            }

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
        }

    private:
        Ruckig<3>
            ruckig;
        Ruckig<3>
            gui_ruckig;
        InputParameter<3> input;
        OutputParameter<3> output;
        Robot robot;
        RobotControlMode control_mode = RobotControlMode::PositionHold;

        bool show_settings = true;
        bool show_trajectory = true;
        bool show_field = true;
        bool show_field_info = true;

        double last_time = 0.0;
        int speed_up = 1;

        double position_pid_kp = 46.67;
        double position_pid_ki = 0;
        double position_pid_kd = 0.22;
        double rotation_pid_kp = 51.48;
        double rotation_pid_ki = 0;
        double rotation_pid_kd = 0.05;
        PID position_pid_x;
        PID position_pid_y;
        PID position_pid_r;

        std::vector<std::pair<float, float>> trail_points{max_trail_points};
        std::vector<std::pair<float, float>> trail_points_old{max_trail_points};
        std::vector<std::pair<float, float>> trail_points_old_old{max_trail_points};
        std::vector<std::pair<float, float>> trail_points_future;

        void init()
        {
            input.duration_discretization = DurationDiscretization::Discrete;

            input.per_dof_synchronization = {Synchronization::Phase, Synchronization::Phase, Synchronization::TimeIfNecessary};

            input.max_velocity = {4.0, 4.0, 4.0};
            input.max_acceleration = {10.0, 10.0, 6.46};
            input.max_jerk = {20.0, 20.0, 6.37};

            input.target_position = {4.0, 4.0, 4.0};
        }

        void resetRobot()
        {
            robot.setPosition(0.0, 0.0, 0.0);
            input.current_position = {0.0, 0.0, 0.0};
            input.current_velocity = {0.0, 0.0, 0.0};
            input.current_acceleration = {0.0, 0.0, 0.0};
            position_pid_x.reset();
            position_pid_y.reset();
            position_pid_r.reset();

            trail_points_old_old = std::move(trail_points_old);
            trail_points_old = std::move(trail_points);
            trail_points.clear();
        }

        void drawSettings()
        {
            if (ImGui::CollapsingHeader("Configuration", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Text("Degrees of Freedom: %zu", input.degrees_of_freedom);

                double min_delta_time = 0.001, max_delta_time = 1.0;
                ImGui::TextUnformatted("Delta Time");
                ImGui::SameLine();
                ImGui::SliderScalar("##delta_time", ImGuiDataType_Double, &ruckig.delta_time, &min_delta_time, &max_delta_time, "%.3f");

                if (gui_ruckig.delta_time != ruckig.delta_time)
                {
                    gui_ruckig.delta_time = ruckig.delta_time;
                }

                ImGui::Spacing();

                // Dropdown for control mode
                static const char *control_options[] = {"PositionHold", "Velocity"};
                int current_control_mode = static_cast<int>(control_mode);

                ImGui::TextUnformatted("Control Mode");
                if (ImGui::BeginCombo("##control-mode", control_options[current_control_mode]))
                {
                    for (int n = 0; n < IM_ARRAYSIZE(control_options); n++)
                    {
                        bool is_selected = (current_control_mode == n);
                        if (ImGui::Selectable(control_options[n], is_selected))
                        {
                            current_control_mode = n;
                        }
                        if (is_selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                        if (ImGui::IsItemHovered())
                        {
                            switch (n)
                            {
                            case 0:
                                ImGui::SetTooltip("Robot follows position using PID (trajectory acts as motion profile) (Default)");
                                break;
                            case 1:
                                ImGui::SetTooltip("Robot follows velocity (trajectory acts as feedback control)");
                                break;
                            }
                        }
                    }
                    ImGui::EndCombo();

                    control_mode = static_cast<RobotControlMode>(current_control_mode);
                }

                if (control_mode == RobotControlMode::PositionHold)
                {
                    {
                        ImGui::Spacing();
                        ImGui::SeparatorText("Position PID Gains");
                        double min_kp = 0.01, max_kp = 115.0;
                        ImGui::Text("kP");
                        ImGui::SliderScalar("##position_pid_kp", ImGuiDataType_Double, &position_pid_kp, &min_kp, &max_kp, "%.2f");
                        ImGui::Spacing();
                        double min_ki = 0.0, max_ki = 1.0;
                        ImGui::Text("kI");
                        ImGui::SliderScalar("##position_pid_ki", ImGuiDataType_Double, &position_pid_ki, &min_ki, &max_ki, "%.2f");
                        ImGui::Spacing();
                        double min_kd = 0.0, max_kd = 1.0;
                        ImGui::Text("kD");
                        ImGui::SliderScalar("##position_pid_kd", ImGuiDataType_Double, &position_pid_kd, &min_kd, &max_kd, "%.2f");
                        ImGui::Spacing();
                    }
                    {
                        ImGui::SeparatorText("Rotation PID Gains");
                        double min_kp = 0.01, max_kp = 115.0;
                        ImGui::Text("kP");
                        ImGui::SliderScalar("##rotation_pid_kp", ImGuiDataType_Double, &rotation_pid_kp, &min_kp, &max_kp, "%.2f");
                        ImGui::Spacing();
                        double min_ki = 0.0, max_ki = 1.0;
                        ImGui::Text("kI");
                        ImGui::SliderScalar("##rotation_pid_ki", ImGuiDataType_Double, &rotation_pid_ki, &min_ki, &max_ki, "%.2f");
                        ImGui::Spacing();
                        double min_kd = 0.0, max_kd = 1.0;
                        ImGui::Text("kD");
                        ImGui::SliderScalar("##rotation_pid_kd", ImGuiDataType_Double, &rotation_pid_kd, &min_kd, &max_kd, "%.2f");
                    }
                }
            }

            ImGui::Spacing();

            if (ImGui::CollapsingHeader("Input Parameters", ImGuiTreeNodeFlags_DefaultOpen))
            {

                double min_vel = 0.01, max_vel = 4.0;
                ImGui::Text("Max Velocity");
                ImGui::SliderScalarN("##max_velocity", ImGuiDataType_Double, input.max_velocity.data(), 3, &min_vel, &max_vel, "%.2f");
                ImGui::Spacing();

                double min_acc = 0.01, max_acc = 10.0;
                ImGui::Text("Max Acceleration");
                ImGui::SliderScalarN("##max_acceleration", ImGuiDataType_Double, input.max_acceleration.data(), 3, &min_acc, &max_acc, "%.2f");
                ImGui::Spacing();

                double min_jerk = 0.01, max_jerk = 20.0;
                ImGui::Text("Max Jerk");
                ImGui::SliderScalarN("##max_jerk", ImGuiDataType_Double, input.max_jerk.data(), 3, &min_jerk, &max_jerk, "%.2f");
                ImGui::Spacing();

                // Dropdown for input.synchronization options
                static const char *sync_options[] = {"Time", "TimeIfNecessary", "Phase", "None"};

                ImGui::TextUnformatted("Synchronization (per DoF)");
                float total_width = ImGui::GetContentRegionAvail().x;
                float combo_width = total_width / 3.0f - 14.0f; // Subtract a few pixels for spacing
                for (int i = 0; i < 3; ++i)
                {
                    if (i > 0)
                        ImGui::SameLine();
                    int current_sync = static_cast<int>(input.per_dof_synchronization.value()[i]);
                    std::string combo_label = "##synchronization_" + std::to_string(i);
                    ImGui::SetNextItemWidth(combo_width);
                    if (ImGui::BeginCombo(combo_label.c_str(), sync_options[current_sync]))
                    {
                        for (int n = 0; n < IM_ARRAYSIZE(sync_options); n++)
                        {
                            bool is_selected = (current_sync == n);
                            if (ImGui::Selectable(sync_options[n], is_selected))
                            {
                                input.per_dof_synchronization.value()[i] = static_cast<Synchronization>(n);
                            }
                            if (is_selected)
                            {
                                ImGui::SetItemDefaultFocus();
                            }
                            if (ImGui::IsItemHovered())
                            {
                                switch (n)
                                {
                                case 0:
                                    ImGui::SetTooltip("Always synchronize the DoFs to reach the target at the same time (Default)");
                                    break;
                                case 1:
                                    ImGui::SetTooltip("Synchronize only when necessary (e.g. for non-zero target velocity or acceleration)");
                                    break;
                                case 2:
                                    ImGui::SetTooltip("Phase synchronize the DoFs when this is possible, else fallback to \"Time\" strategy.\nPhase synchronization will result a straight-line trajectory");
                                    break;
                                case 3:
                                    ImGui::SetTooltip("Calculate every DoF independently");
                                    break;
                                }
                            }
                        }
                        ImGui::EndCombo();
                    }
                }

                ImGui::Spacing();

                // Dropdown for input.duration_discretization options
                static const char *duration_discretization_options[] = {"Continuous", "Discrete"};
                int current_duration_discretization = static_cast<int>(input.duration_discretization);

                ImGui::TextUnformatted("Duration Discretization");
                if (ImGui::BeginCombo("##duration_discretization", duration_discretization_options[current_duration_discretization]))
                {
                    for (int n = 0; n < IM_ARRAYSIZE(duration_discretization_options); n++)
                    {
                        bool is_selected = (current_duration_discretization == n);
                        if (ImGui::Selectable(duration_discretization_options[n], is_selected))
                        {
                            current_duration_discretization = n;
                        }
                        if (is_selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                        if (ImGui::IsItemHovered())
                        {
                            switch (n)
                            {
                            case 0:
                                ImGui::SetTooltip("Every trajectory synchronization duration is allowed (Default)");
                                break;
                            case 1:
                                ImGui::SetTooltip("The trajectory synchronization duration must be a multiple of the control cycle");
                                break;
                            }
                        }
                    }
                    ImGui::EndCombo();

                    input.duration_discretization = static_cast<DurationDiscretization>(current_duration_discretization);
                }
            }
        }

        void drawTrajectory()
        {
            InputParameter<3> temp_input{input};
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

            float range = global_max - global_min > 1e-6f ? (global_max - global_min) : 1.0f;

            const float graph_height = 420.0f;
            const float graph_width = ImGui::GetContentRegionAvail().x;
            const int N = static_cast<int>(p0.size());

            auto draw_multi_line = [&](const char *label, const std::vector<float> &pos, const std::vector<float> &vel, const std::vector<float> &acc)
            {
                ImGui::Text("%s", label);
                ImVec2 p0 = ImGui::GetCursorScreenPos();
                ImDrawList *draw_list = ImGui::GetWindowDrawList();
                ImVec2 size = ImVec2(graph_width, graph_height);
                draw_list->AddRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), IM_COL32(200, 200, 200, 255));

                auto plot_line = [&](const std::vector<float> &data, ImU32 color)
                {
                    for (int i = 1; i < N; ++i)
                    {
                        float x0 = p0.x + (i - 1) * size.x / (N - 1);
                        float x1 = p0.x + i * size.x / (N - 1);
                        float y0 = p0.y + size.y - ((data[i - 1] - global_min) / range) * size.y;
                        float y1 = p0.y + size.y - ((data[i] - global_min) / range) * size.y;
                        draw_list->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), color, 2.0f);
                    }
                };
                plot_line(pos, IM_COL32(43, 60, 240, 255)); // Position - blue
                plot_line(vel, IM_COL32(255, 102, 0, 255)); // Velocity - orange
                plot_line(acc, IM_COL32(42, 181, 0, 255));  // Acceleration - green
                ImGui::Dummy(size);
            };

            draw_multi_line("Axis 0 (blue=pos, orange=vel, green=acc)", p0, v0, a0);
            draw_multi_line("Axis 1 (blue=pos, orange=vel, green=acc)", p1, v1, a1);
            draw_multi_line("Axis 2 (blue=pos, orange=vel, green=acc)", p2, v2, a2);
        }

        void drawField()
        {
            // Define field bounds
            const float field_min_x = -5.0f, field_max_x = 5.0f;
            const float field_min_y = -5.0f, field_max_y = 5.0f;

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

            ImDrawList *draw_list = ImGui::GetWindowDrawList();
            ImVec2 draw_offset = canvas_pos;
            // Center the field in the canvas
            draw_offset.x += (canvas_size.x - draw_size.x) * 0.5f;
            draw_offset.y += (canvas_size.y - draw_size.y) * 0.5f;

            // Handle right-click to set target position
            ImGui::InvisibleButton("field_canvas", canvas_size, ImGuiButtonFlags_MouseButtonRight);
            bool hovered = ImGui::IsItemHovered();
            bool right_clicked = hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right);
            if (right_clicked)
            {
                ImVec2 mouse_pos = ImGui::GetIO().MousePos;
                // Convert mouse_pos to field coordinates
                float px = mouse_pos.x - draw_offset.x;
                float py = mouse_pos.y - draw_offset.y;
                // Clamp to draw_size
                px = std::clamp(px, 0.0f, draw_size.x);
                py = std::clamp(py, 0.0f, draw_size.y);
                // Convert to normalized [0,1]
                float norm_x = px / draw_size.x;
                float norm_y = 1.0f - (py / draw_size.y); // y axis flipped
                // Convert to field coordinates
                float field_x = field_min_x + norm_x * field_width;
                float field_y = field_min_y + norm_y * field_height;
                // Clamp to field bounds
                field_x = std::clamp(field_x, field_min_x, field_max_x);
                field_y = std::clamp(field_y, field_min_y, field_max_y);
                // Set target position (keep z/rotation unchanged)
                input.target_position[0] = field_x;
                input.target_position[1] = field_y;
                // Optionally, keep input.target_position[2] unchanged
            }

            // Draw field border
            draw_list->AddRect(draw_offset, ImVec2(draw_offset.x + draw_size.x, draw_offset.y + draw_size.y), IM_COL32(200, 200, 200, 255), 0.0f, 0, 2.0f);

            auto drawRobot = [&](ImDrawList *draw_list, float rx, float ry, float rot, bool is_target)
            {
                // Map robot position to field coordinates
                float norm_x = (rx - field_min_x) / field_width;
                float norm_y = (ry - field_min_y) / field_height;
                float px = draw_offset.x + norm_x * draw_size.x;
                float py = draw_offset.y + (1.0f - norm_y) * draw_size.y; // y axis flipped for screen

                // Draw rotated square (robot)
                float half_size = 0.05f * draw_size.x; // pixels, half side length
                ImVec2 corners[4];
                for (int i = 0; i < 4; ++i)
                {
                    float angle = rot + M_PI_4 + i * M_PI_2;
                    corners[i].x = px + half_size * cosf(angle);
                    corners[i].y = py + half_size * sinf(angle);
                }
                draw_list->AddConvexPolyFilled(corners, 4, is_target ? IM_COL32(240, 60, 43, 180) : IM_COL32(43, 60, 240, 180));
                draw_list->AddPolyline(corners, 4, is_target ? IM_COL32(240, 60, 43, 255) : IM_COL32(43, 60, 240, 255), true, 2.0f);

                // Optionally, draw a heading line
                float heading_len = half_size * 1.2f;
                ImVec2 heading_end = ImVec2(px + heading_len * cosf(rot), py + heading_len * sinf(rot));
                draw_list->AddLine(ImVec2(px, py), heading_end, is_target ? IM_COL32(0, 102, 255, 255) : IM_COL32(255, 102, 0, 255), 2.5f);
            };

            // Get robot position and rotation
            double dx = 0;
            double dy = 0;
            double dr = 0;
            robot.getPosition(dx, dy, dr);
            drawRobot(draw_list, static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dr), false);
            drawRobot(draw_list, static_cast<float>(input.target_position[0]), static_cast<float>(input.target_position[1]), static_cast<float>(input.target_position[2]), true);
            drawRobot(draw_list, static_cast<float>(output.new_position[0]), static_cast<float>(output.new_position[1]), static_cast<float>(output.new_position[2]), true);

            if (trail_points.empty() ||
                std::hypot(static_cast<float>(dx) - trail_points.back().first, static_cast<float>(dy) - trail_points.back().second) > 0.01f)
            {
                if (trail_points.size() < max_trail_points)
                {
                    trail_points.emplace_back(static_cast<float>(dx), static_cast<float>(dy));
                }
                else
                {
                    trail_points.erase(trail_points.begin());
                    trail_points.emplace_back(static_cast<float>(dx), static_cast<float>(dy));
                }
            }

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
                        float px0 = draw_offset.x + norm_x0 * draw_size.x;
                        float py0 = draw_offset.y + (1.0f - norm_y0) * draw_size.y;

                        float norm_x1 = (rx1 - field_min_x) / field_width;
                        float norm_y1 = (ry1 - field_min_y) / field_height;
                        float px1 = draw_offset.x + norm_x1 * draw_size.x;
                        float py1 = draw_offset.y + (1.0f - norm_y1) * draw_size.y;

                        draw_list->AddLine(ImVec2(px0, py0), ImVec2(px1, py1), IM_COL32(255, 255, 255, alpha), 2.0f);
                    }
                }
            };

            drawTrail(trail_points, 180);        // Draw current trail with alpha 180
            drawTrail(trail_points_old, 120);    // Draw old trail with alpha 120
            drawTrail(trail_points_old_old, 60); // Draw older trail with alpha 60
            drawTrail(trail_points_future, 30);  // Draw future trail with alpha 30
        }

        void drawFieldInfo()
        {
            ImGui::Text("Field Size: %.1f x %.1f", 10.0f, 10.0f);
            ImGui::Text("Robot Size: %.2f m", 0.1f);
            double dx = 0;
            double dy = 0;
            double dr = 0;
            robot.getPosition(dx, dy, dr);
            ImGui::Text("Robot Position: (%.2f, %.2f, %.2f)", dx, dy, dr);
            ImGui::Text("Target Position: (%.2f, %.2f, %.2f)", input.target_position[0], input.target_position[1], input.target_position[2]);
            ImGui::Text("Current Velocity: (%.2f, %.2f, %.2f)", input.current_velocity[0], input.current_velocity[1], input.current_velocity[2]);
            ImGui::Text("Current Acceleration: (%.2f, %.2f, %.2f)", input.current_acceleration[0], input.current_acceleration[1], input.current_acceleration[2]);
            ImGui::Spacing();

            ImGui::Text("Speed Up");
            ImGui::SameLine();
            ImGui::SliderInt("##speed_up", &speed_up, 1, 20, "%d", ImGuiSliderFlags_AlwaysClamp);
        }
    };
}