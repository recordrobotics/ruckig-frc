module;

#include <spdlog/spdlog.h>
#include <imgui.h>
#include <ruckig/ruckig.hpp>
#include <math.h>
#include <algorithm>
#include <cfloat>

export module ui.ruckig;

import ui.uimodule;
import robot;
import pid;
import random;

using namespace ruckig;

const int max_iterations = 10000;     // Maximum iterations for trajectory calculation
const size_t max_trail_points = 5000; // Maximum number of trail points

export namespace ui
{
    enum class RobotControlMode
    {
        PID,
        Feedforward,
        Combined
    };

    enum class WaypointMode
    {
        FullStop,
        LinearVelocity,
        LinearAcceleration,
        OptimizedPolynomial
    };

    class RuckigModule : public UIModule
    {
    public:
        RuckigModule() : ruckig(0.02), gui_ruckig(0.02), robot()
        {
            spdlog::info("Ruckig Module initialized with {} DoFs and {} delta time", ruckig.degrees_of_freedom, ruckig.delta_time);

            next_delta_time = ruckig.delta_time;
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
                ImGui::MenuItem("Show Waypoints", nullptr, &show_waypointlist);

                if (ImGui::MenuItem("Reset"))
                {
                    ruckig.reset();
                    gui_ruckig.reset();
                    ruckig.delta_time = 0.02; // Reset delta time
                    next_delta_time = ruckig.delta_time;
                    gui_ruckig.delta_time = ruckig.delta_time;
                    input = InputParameter<3>();
                    output = OutputParameter<3>();
                    init();

                    spdlog::info("Ruckig reset to initial state");
                }

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Waypoints"))
            {
                if (ImGui::MenuItem("Clear"))
                {
                    waypoints.clear();
                }

                if (ImGui::MenuItem("Add Triangle"))
                {
                    addWaypoint(-2.58, 2.9, 1.11);
                    addWaypoint(-2.82, -2.81, -1.2);
                    addWaypoint(3.17, 0.14, -2.61);
                }

                if (ImGui::MenuItem("Add Circle"))
                {
                    for (int i = 0; i < 10; i++)
                    {
                        double angle = i / 10.0 * 2 * M_PI; // Spread waypoints in a circle
                        double x = 5 * cos(angle);
                        double y = 5 * sin(angle);
                        double r = rnd::random_double(-3.14, 3.14);

                        addWaypoint(x, y, r);
                    }
                }

                if (ImGui::MenuItem("Random Jitter"))
                {
                    for (auto &waypoint : waypoints)
                    {
                        waypoint.x += rnd::random_double(-0.5, 0.5);
                        waypoint.y += rnd::random_double(-0.5, 0.5);
                        waypoint.r += rnd::random_double(-0.5, 0.5);
                        waypoint.x = std::clamp(waypoint.x, -5.0, 5.0);
                        waypoint.y = std::clamp(waypoint.y, -5.0, 5.0);
                        waypoint.r = std::fmod(waypoint.r + M_PI, 2 * M_PI) - M_PI; // Normalize to [-π, π]
                        fitWaypoints();
                    }
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

            last_delta_time = next_delta_time;

            double current_time = ImGui::GetTime();
            if (next_delta_time > 0.0 && (current_time - last_time) >= next_delta_time)
            {
                last_time = current_time;
                updateStep();
                next_delta_time = ruckig.delta_time +
                                  (rnd::random_double(0, 1) < loop_overrun_frequency ? rnd::random_double(0, loop_overrun_amount) : 0); // Simulate loop overrun
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

            if (show_waypointlist)
            {
                ImGui::SetNextWindowPos(ImVec2(600, 600), ImGuiCond_FirstUseEver);
                if (ImGui::Begin("Waypoints", &show_waypointlist))
                {
                    drawWaypointList();
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
        RobotControlMode control_mode = RobotControlMode::Combined;
        WaypointMode waypoint_mode = WaypointMode::LinearAcceleration;

        bool show_settings = true;
        bool show_trajectory = true;
        bool show_field = true;
        bool show_field_info = true;
        bool show_waypointlist = true;

        double last_time = 0.0;
        int speed = 1;

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

        std::vector<float> robotErrorP, robotErrorV, robotErrorA;
        std::vector<float> targetX, targetY, targetR, currentX, currentY, currentR;

        double last_delta_time = 0.02;
        double next_delta_time = 0.02;

        double loop_overrun_frequency = 0.0;
        double loop_overrun_amount = 0.01;
        double position_noise_amount = 0.1;
        double rotation_noise_amount = 0.1;

        typedef struct Pose2d
        {
            double x, y, r;
            double vx, vy, vr;
            double ax, ay, ar;
            bool editing = false; // Flag to indicate if the waypoint is being edited
            ImColor color;
        } Pose2d;

        std::vector<Pose2d> waypoints;
        int current_waypoint = -1;

        void init()
        {
            input.duration_discretization = DurationDiscretization::Discrete;

            input.per_dof_synchronization = {Synchronization::Phase, Synchronization::Phase, Synchronization::None};

            input.max_velocity = {4.0, 4.0, 4.0};
            input.max_acceleration = {10.0, 10.0, 10.0};
            input.max_jerk = {20.0, 20.0, 20.0};

            input.target_position = {0.0, 0.0, 0.0};

            waypoints.clear();
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

            robotErrorP.clear();
            robotErrorV.clear();
            robotErrorA.clear();

            targetX.clear();
            targetY.clear();
            targetR.clear();
            currentX.clear();
            currentY.clear();
            currentR.clear();

            fitWaypoints();
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
                    next_delta_time = ruckig.delta_time;
                }

                ImGui::Spacing();

                double min_loop_overrun_frequency = 0.0, max_loop_overrun_frequency = 1.0;
                ImGui::TextUnformatted("Loop Overrun Frequency");
                {
                    double loop_overrun_frequency_pct = loop_overrun_frequency * 100.0;
                    double min_pct = min_loop_overrun_frequency * 100.0;
                    double max_pct = max_loop_overrun_frequency * 100.0;
                    if (ImGui::SliderScalar("##loop_overrun_frequency", ImGuiDataType_Double,
                                            &loop_overrun_frequency_pct, &min_pct, &max_pct, "%.1f%%"))
                    {
                        loop_overrun_frequency = loop_overrun_frequency_pct / 100.0;
                    }
                }

                double min_loop_overrun_amount = 0, max_loop_overrun_amount = 0.5;
                ImGui::TextUnformatted("Loop Overrun Amount");
                ImGui::SliderScalar("##loop_overrun_amount", ImGuiDataType_Double, &loop_overrun_amount, &min_loop_overrun_amount, &max_loop_overrun_amount, "%.3f");

                ImGui::TextUnformatted("Noise Amount");
                {
                    ImGuiStyle &style = ImGui::GetStyle();
                    const char *label_xy = "XY";
                    const char *label_r = "R";

                    float avail = ImGui::GetContentRegionAvail().x;
                    float label1_w = ImGui::CalcTextSize(label_xy).x;
                    float label2_w = ImGui::CalcTextSize(label_r).x;
                    // 4 inline items => 3 spacings
                    float total_spacing = style.ItemSpacing.x * 3.0f;
                    float remaining = avail - label1_w - label2_w - total_spacing;
                    if (remaining < 0.0f)
                        remaining = 0.0f;
                    float slider_w = remaining * 0.5f;

                    // Minimum practical width
                    slider_w = std::max(slider_w, 50.0f);

                    ImGui::TextUnformatted(label_xy);
                    ImGui::SameLine();
                    double min_position_noise = 0, max_position_noise = 1.0;
                    ImGui::SetNextItemWidth(slider_w);
                    ImGui::SliderScalar("##position_noise_amount", ImGuiDataType_Double,
                                        &position_noise_amount, &min_position_noise, &max_position_noise, "%.3f");

                    ImGui::SameLine();
                    ImGui::TextUnformatted(label_r);
                    ImGui::SameLine();
                    double min_rotation_noise = 0, max_rotation_noise = 0.5;
                    ImGui::SetNextItemWidth(slider_w);
                    ImGui::SliderScalar("##rotation_noise_amount", ImGuiDataType_Double,
                                        &rotation_noise_amount, &min_rotation_noise, &max_rotation_noise, "%.3f");
                }

                ImGui::Spacing();

                // Dropdown for waypoint mode
                static const char *waypoint_options[] = {"Full Stop", "Linear Velocity", "Linear Acceleration", "Optimized Polynomial"};
                int current_waypoint_mode = static_cast<int>(waypoint_mode);

                ImGui::TextUnformatted("Waypoint Mode");
                if (ImGui::BeginCombo("##waypoint-mode", waypoint_options[current_waypoint_mode]))
                {
                    for (int n = 0; n < IM_ARRAYSIZE(waypoint_options); n++)
                    {
                        bool is_selected = (current_waypoint_mode == n);
                        if (ImGui::Selectable(waypoint_options[n], is_selected))
                        {
                            current_waypoint_mode = n;
                            waypoint_mode = static_cast<WaypointMode>(current_waypoint_mode);
                            fitWaypoints();
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
                                ImGui::SetTooltip("Velocity and Acceleration are zero at each waypoint");
                                break;
                            case 1:
                                ImGui::SetTooltip("Velocity is calculated using linear interpolation between waypoints. Acceleration is zero. (Use this if Linear Acceleration is too unstable)");
                                break;
                            case 2:
                                ImGui::SetTooltip("Velocity and Acceleration are calculated using linear interpolation between waypoints. (Might produce more optimal trajectories than Linear Velocity but more unstable) (Default)");
                                break;
                            case 3:
                                ImGui::SetTooltip("Velocity and Acceleration are time-optimized using cubic polynomial segments. (Sometimes unstable)");
                                break;
                            }
                        }
                    }
                    ImGui::EndCombo();
                }

                // Dropdown for control mode
                static const char *control_options[] = {"PID", "Feedforward", "Combined"};
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
                                ImGui::SetTooltip("Robot follows position using PID (trajectory acts as motion profile)");
                                break;
                            case 1:
                                ImGui::SetTooltip("Robot follows velocity (trajectory acts as feedback control)");
                                break;
                            case 2:
                                ImGui::SetTooltip("Robot follows position using PID and velocity using feedforward (trajectory acts as motion profile and feedback control) (Default)");
                                break;
                            }
                        }
                    }
                    ImGui::EndCombo();

                    control_mode = static_cast<RobotControlMode>(current_control_mode);
                }

                if (control_mode == RobotControlMode::PID || control_mode == RobotControlMode::Combined)
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

            drawMultiLine("Current vs Target (blue=x, orange=y, green=rot)",
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

                // Set waypoint position for all editing waypoints
                for (auto &wp : waypoints)
                {
                    if (wp.editing)
                    {
                        wp.x = field_x;
                        wp.y = field_y;
                        fitWaypoints();
                    }
                }

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

            // Draw field border
            draw_list->AddRect(draw_offset, ImVec2(draw_offset.x + draw_size.x, draw_offset.y + draw_size.y), IM_COL32(200, 200, 200, 255), 0.0f, 0, 2.0f);

            auto drawRobot = [&](ImDrawList *draw_list, float rx, float ry, float rot, ImColor color)
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

                draw_list->AddConvexPolyFilled(corners, 4, IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 180));
                draw_list->AddPolyline(corners, 4, IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 255), true, 2.0f);

                // Optionally, draw a heading line
                float heading_len = half_size * 1.2f;
                ImVec2 heading_end = ImVec2(px + heading_len * cosf(rot), py + heading_len * sinf(rot));
                draw_list->AddLine(ImVec2(px, py), heading_end, IM_COL32(102, 102, 255, 255), 2.5f);
            };

            auto drawArrow = [&](ImDrawList *draw_list, float x, float y, float dx, float dy, ImColor color)
            {
                // Convert field coordinates to screen coordinates for start point
                float norm_x = (x - field_min_x) / field_width;
                float norm_y = (y - field_min_y) / field_height;
                float px = draw_offset.x + norm_x * draw_size.x;
                float py = draw_offset.y + (1.0f - norm_y) * draw_size.y; // y axis flipped

                // Convert field coordinates to screen coordinates for end point
                float end_x = x + dx;
                float end_y = y + dy;
                float end_norm_x = (end_x - field_min_x) / field_width;
                float end_norm_y = (end_y - field_min_y) / field_height;
                float end_px = draw_offset.x + end_norm_x * draw_size.x;
                float end_py = draw_offset.y + (1.0f - end_norm_y) * draw_size.y; // y axis flipped

                // Draw arrow line
                draw_list->AddLine(ImVec2(px, py), ImVec2(end_px, end_py), IM_COL32(static_cast<int>(color.Value.x * 255), static_cast<int>(color.Value.y * 255), static_cast<int>(color.Value.z * 255), 255), 2.0f);

                // Calculate arrow head
                float arrow_length = sqrtf((end_px - px) * (end_px - px) + (end_py - py) * (end_py - py));
                if (arrow_length > 10.0f) // Only draw arrow head if line is long enough
                {
                    float arrow_head_size = 8.0f;
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
            double dx, dy, dr;
            robot.getState(&dx, &dy, &dr);
            drawRobot(draw_list, static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dr), ImColor(43, 60, 240));

            for (auto &wp : waypoints)
            {
                drawRobot(draw_list, static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.r), wp.color);
                drawArrow(draw_list, static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.vx * 0.3), static_cast<float>(wp.vy * 0.3), wp.color);
                drawArrow(draw_list, static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.ax * 0.3), static_cast<float>(wp.ay * 0.3), wp.color);
            }

            drawRobot(draw_list, static_cast<float>(output.new_position[0]), static_cast<float>(output.new_position[1]), static_cast<float>(output.new_position[2]), ImColor(240, 60, 43));

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
            double dx, dy, dr;
            robot.getState(&dx, &dy, &dr);
            ImGui::Text("Robot Position: (%.2f, %.2f, %.2f)", dx, dy, dr);
            ImGui::Text("Target Position: (%.2f, %.2f, %.2f)", input.target_position[0], input.target_position[1], input.target_position[2]);
            ImGui::Text("Current Velocity: (%.2f, %.2f, %.2f)", input.current_velocity[0], input.current_velocity[1], input.current_velocity[2]);
            ImGui::Text("Current Acceleration: (%.2f, %.2f, %.2f)", input.current_acceleration[0], input.current_acceleration[1], input.current_acceleration[2]);
            ImGui::Spacing();

            ImGui::Text("Delta Time: %.3f s", last_delta_time);
            ImGui::Spacing();

            ImGui::Text("Speed");
            ImGui::SameLine();
            ImGui::SliderInt("##speed", &speed, 0, 20, "%dx", ImGuiSliderFlags_AlwaysClamp);
        }

        void drawWaypointList()
        {
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

                // Add X button
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(220, 60, 60, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(255, 80, 80, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(180, 40, 40, 255));
                if (ImGui::SmallButton("X"))
                {
                    waypoints.erase(waypoints.begin() + i);
                    ImGui::PopStyleColor(3);
                    ImGui::PopID();
                    fitWaypoints();
                    break; // Exit loop to avoid issues with modified vector
                }
                ImGui::PopStyleColor(3);

                ImGui::PopID();
            }
            if (ImGui::Button("Add Waypoint"))
            {
                addWaypoint(input.current_position[0],
                            input.current_position[1],
                            input.current_position[2]);
            }
        }

        inline void addWaypoint(double x, double y, double r)
        {
            float hue = waypoints.empty() ? 0.0f : (rgbToHue(waypoints.back().color) + 0.05f);
            Pose2d wp{
                x, y, r,
                0, 0, 0,
                0, 0, 0,
                false,
                ImColor::HSV(hue, 1.0f, 1.0f)};
            waypoints.push_back(wp);
            fitWaypoints();
        }

        Result ruckigResult = Result::Working;

        void updateStep()
        {
            if (waypoints.empty())
                return;

            for (int i = 0; i < speed; i++)
            {
                double cx, cy, cr, cvx, cvy, cvr, cax, cay, car;
                robot.getState(&cx, &cy, &cr, &cvx, &cvy, &cvr, &cax, &cay, &car);

                if (current_waypoint == -1)
                {
                    fitWaypoints();
                    current_waypoint = 0;
                    input.current_position[0] = cx;
                    input.current_position[1] = cy;
                    input.current_position[2] = cr;
                    input.current_velocity[0] = cvx;
                    input.current_velocity[1] = cvy;
                    input.current_velocity[2] = cvr;
                    input.current_acceleration[0] = cax;
                    input.current_acceleration[1] = cay;
                    input.current_acceleration[2] = car;
                }

                input.target_position[0] = waypoints[current_waypoint].x;
                input.target_position[1] = waypoints[current_waypoint].y;
                input.target_position[2] = waypoints[current_waypoint].r;
                input.target_velocity[0] = waypoints[current_waypoint].vx;
                input.target_velocity[1] = waypoints[current_waypoint].vy;
                input.target_velocity[2] = waypoints[current_waypoint].vr;
                input.target_acceleration[0] = waypoints[current_waypoint].ax;
                input.target_acceleration[1] = waypoints[current_waypoint].ay;
                input.target_acceleration[2] = waypoints[current_waypoint].ar;

                // Calculate trajectory
                // if (ruckigResult == Result::ErrorSynchronizationCalculation)
                // {
                //     input.synchronization = Synchronization::None;
                //     input.per_dof_synchronization = {Synchronization::None, Synchronization::None, Synchronization::None};
                // }
                ruckigResult = ruckig.update(input, output);
                output.pass_to_input(input);

                double x = output.new_position[0];
                double y = output.new_position[1];
                double r = output.new_position[2];
                double vx = 0, vy = 0, vr = 0;

                if (ruckigResult != Result::Finished)
                {
                    double ex = x - cx;
                    double ey = y - cy;
                    double er = r - cr;
                    double evx = output.new_velocity[0] - cvx;
                    double evy = output.new_velocity[1] - cvy;
                    double evr = output.new_velocity[2] - cvr;
                    double eax = output.new_acceleration[0] - cax;
                    double eay = output.new_acceleration[1] - cay;
                    double ear = output.new_acceleration[2] - car;

                    robotErrorP.push_back(static_cast<float>(std::max(std::abs(ex), std::max(std::abs(ey), std::abs(er)))));
                    robotErrorV.push_back(static_cast<float>(std::max(std::abs(evx), std::max(std::abs(evy), std::abs(evr)))));
                    robotErrorA.push_back(static_cast<float>(std::max(std::abs(eax), std::max(std::abs(eay), std::abs(ear)))));

                    targetX.push_back(static_cast<float>(x));
                    targetY.push_back(static_cast<float>(y));
                    targetR.push_back(static_cast<float>(r));
                    currentX.push_back(static_cast<float>(cx));
                    currentY.push_back(static_cast<float>(cy));
                    currentR.push_back(static_cast<float>(cr));
                }

                if (ruckigResult != Result::Working)
                {
                    // Move to next waypoint if close enough to current target
                    double distance_to_target = std::hypot(x - input.target_position[0], y - input.target_position[1]);
                    if (distance_to_target < 0.05 || ruckigResult == Result::Finished)
                    {
                        current_waypoint++;
                        if (current_waypoint >= waypoints.size())
                        {
                            current_waypoint = -1;
                        }
                    }
                    else if (ruckigResult != Result::Finished)
                    {
                        spdlog::warn("Ruckig update failed: {}", std::to_string(ruckigResult));
                        input.current_position[0] = cx;
                        input.current_position[1] = cy;
                        input.current_position[2] = cr;
                        input.current_velocity[0] = cvx;
                        input.current_velocity[1] = cvy;
                        input.current_velocity[2] = cvr;
                        input.current_acceleration[0] = cax;
                        input.current_acceleration[1] = cay;
                        input.current_acceleration[2] = car;

                        if (ruckigResult == Result::ErrorInvalidInput)
                        {
                            input.control_interface = ControlInterface::Velocity;
                            ruckigResult = ruckig.update(input, output);
                            output.pass_to_input(input);
                            input.control_interface = ControlInterface::Position;
                        }
                    }
                }

                if (control_mode == RobotControlMode::PID || control_mode == RobotControlMode::Combined)
                {
                    // PID uses ruckig delta time because next_delta_time simulates loop overrun which doesn't change the internal PID delta time
                    vx += position_pid_x.calculate(x, cx, position_pid_kp, position_pid_ki, position_pid_kd, ruckig.delta_time);
                    vy += position_pid_y.calculate(y, cy, position_pid_kp, position_pid_ki, position_pid_kd, ruckig.delta_time);
                    vr += position_pid_r.calculate(r, cr, rotation_pid_kp, rotation_pid_ki, rotation_pid_kd, ruckig.delta_time);
                }

                if (control_mode == RobotControlMode::Feedforward || control_mode == RobotControlMode::Combined)
                {
                    vx += output.new_velocity[0];
                    vy += output.new_velocity[1];
                    vr += output.new_velocity[2];
                }

                vx += getPositionNoise(vx);
                vy += getPositionNoise(vy);
                vr += getRotationNoise(vr);

                double ax = 0;
                double ay = 0;
                double ar = 0;

                // Robot update uses next_delta_time because this is where the actual physics are simulated
                robot.update(next_delta_time, x, y, r, vx, vy, vr,
                             ax, ay, ar);

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
        }

        inline double getPositionNoise(double velocity)
        {
            return rnd::random_double(-position_noise_amount, position_noise_amount) * std::abs(velocity);
        }

        inline double getRotationNoise(double velocity)
        {
            return rnd::random_double(-position_noise_amount, position_noise_amount) * std::abs(velocity);
        }

        inline float rgbToHue(const ImColor &color)
        {
            float h, s, v;
            ImGui::ColorConvertRGBtoHSV(color.Value.x, color.Value.y, color.Value.z, h, s, v);
            return h;
        }

        inline void fitWaypoints()
        {
            // reset velocities and accelerations
            for (auto &wp : waypoints)
            {
                wp.vx = 0.0;
                wp.vy = 0.0;
                wp.vr = 0.0;
                wp.ax = 0.0;
                wp.ay = 0.0;
                wp.ar = 0.0;
            }

            switch (waypoint_mode)
            {
            case WaypointMode::FullStop:
                // No fitting needed, waypoints are already set to full stop
                break;
            case WaypointMode::LinearVelocity:
                fitLinearWaypoint(input.current_position, waypoints, input.max_velocity, input.max_acceleration, false);
                break;
            case WaypointMode::LinearAcceleration:
                fitLinearWaypoint(input.current_position, waypoints, input.max_velocity, input.max_acceleration, true);
                break;
            case WaypointMode::OptimizedPolynomial:
                fitTimeOptimalPolynomial(input.current_position, input.current_velocity, input.current_acceleration, waypoints, input.max_velocity, input.max_acceleration, input.max_jerk, max_iterations, input.duration_discretization);
                break;
            }
        }

        static void fitLinearWaypoint(std::array<double, 3> startPos, std::vector<Pose2d> &waypoints, std::array<double, 3> maxVel, std::array<double, 3> maxAcc, bool useAcceleration)
        {
            if (waypoints.empty())
                return;

            for (size_t i = 0; i < waypoints.size() - 1; ++i)
            {
                double px = (i == 0) ? startPos[0] : waypoints[i - 1].x;
                double py = (i == 0) ? startPos[1] : waypoints[i - 1].y;
                double pr = (i == 0) ? startPos[2] : waypoints[i - 1].r;

                double nx = (i < waypoints.size() - 1) ? waypoints[i + 1].x : waypoints[i].x;
                double ny = (i < waypoints.size() - 1) ? waypoints[i + 1].y : waypoints[i].y;
                double nr = (i < waypoints.size() - 1) ? waypoints[i + 1].r : waypoints[i].r;

                double dpx = waypoints[i].x - px;
                double dpy = waypoints[i].y - py;
                double dpr = waypoints[i].r - pr;
                double dnx = nx - waypoints[i].x;
                double dny = ny - waypoints[i].y;
                double dnr = nr - waypoints[i].r;

                double lp = std::hypot(dpx, dpy);
                double ln = std::hypot(dnx, dny);
                double t = lp < ln ? lp / (2 * ln) : (1 - ln / (2 * lp));

                double dx = dnx * t + dpx * (1 - t);
                double dy = dny * t + dpy * (1 - t);
                double dr = dnr * t + dpr * (1 - t);
                dr = std::clamp(dr, -maxVel[2], maxVel[2]);

                double dist = std::hypot(dx, dy);
                if (dist > 1e-6)
                {
                    double max_v = std::hypot(maxVel[0], maxVel[1]);
                    max_v /= std::abs(dr);
                    if (dist > max_v)
                    {
                        dx /= dist;
                        dy /= dist;
                        double v_peak = max_v;
                        dx *= v_peak;
                        dy *= v_peak;
                    }
                }
                else
                {
                    dx = 0.0;
                    dy = 0.0;
                }

                if (useAcceleration)
                {
                    dnr = std::clamp(dnr, -maxAcc[2], maxAcc[2]);
                    double dist_a = std::hypot(dnx, dny);
                    if (dist_a > 1e-6)
                    {
                        double max_a = std::hypot(maxAcc[0], maxAcc[1]);
                        max_a /= std::abs(dnr);
                        if (dist_a > max_a)
                        {
                            dnx /= dist_a;
                            dny /= dist_a;
                            double a_peak = max_a;
                            dnx *= a_peak;
                            dny *= a_peak;
                        }
                    }
                    else
                    {
                        dnx = 0.0;
                        dny = 0.0;
                    }

                    waypoints[i].ax = dnx;
                    waypoints[i].ay = dny;
                    waypoints[i].ar = dnr;
                }

                waypoints[i].vx = dx;
                waypoints[i].vy = dy;
                waypoints[i].vr = dr;
            }
        }

        static void fitTimeOptimalPolynomial(std::array<double, 3> startPos, std::array<double, 3> startVel, std::array<double, 3> startAcc, std::vector<Pose2d> &waypoints, std::array<double, 3> maxVel, std::array<double, 3> maxAcc, std::array<double, 3> maxJerk, double maxIterations, DurationDiscretization discretization)
        {
            if (waypoints.empty())
                return;

            // Structure to hold trajectory segment
            struct TrajectorySegment
            {
                double duration;
                double coeffs[3][4]; // [axis][coefficient] - cubic polynomial coeffs: a0 + a1*t + a2*t^2 + a3*t^3
                double start_pos[3], start_vel[3], start_acc[3];
                double end_pos[3], end_vel[3], end_acc[3];
            };

            std::vector<TrajectorySegment> segments;

            // Current state
            double current_pos[3] = {startPos[0], startPos[1], startPos[2]};
            double current_vel[3] = {startVel[0], startVel[1], startVel[2]};
            double current_acc[3] = {startAcc[0], startAcc[1], startAcc[2]};

            // Process each waypoint
            for (size_t wp_idx = 0; wp_idx < waypoints.size(); ++wp_idx)
            {
                auto &waypoint = waypoints[wp_idx];

                // Target position for this segment
                double target_pos[3] = {waypoint.x, waypoint.y, waypoint.r};

                // Target velocity and acceleration
                double target_vel[3], target_acc[3];

                if (wp_idx == waypoints.size() - 1)
                {
                    // Last waypoint - end with zero velocity and acceleration
                    target_vel[0] = target_vel[1] = target_vel[2] = 0.0;
                    target_acc[0] = target_acc[1] = target_acc[2] = 0.0;
                }
                else
                {
                    // Intermediate waypoint - compute physically realistic and time-optimal velocity
                    const auto &next_waypoint = waypoints[wp_idx + 1];
                    double direction[3] = {
                        next_waypoint.x - waypoint.x,
                        next_waypoint.y - waypoint.y,
                        next_waypoint.r - waypoint.r};

                    // Calculate segment length and direction
                    double segment_length = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);

                    // Compute path curvature to determine safe cornering velocity
                    double curvature = 0.0;
                    if (wp_idx > 0 && segment_length > 1e-6)
                    {
                        const auto &prev_waypoint = waypoints[wp_idx - 1];
                        double prev_direction[3] = {
                            waypoint.x - prev_waypoint.x,
                            waypoint.y - prev_waypoint.y,
                            waypoint.r - prev_waypoint.r};

                        double prev_length = std::sqrt(prev_direction[0] * prev_direction[0] + prev_direction[1] * prev_direction[1] + prev_direction[2] * prev_direction[2]);

                        if (prev_length > 1e-6)
                        {
                            // Normalize directions
                            for (int i = 0; i < 3; ++i)
                            {
                                direction[i] /= segment_length;
                                prev_direction[i] /= prev_length;
                            }

                            // Calculate angle change (approximation of curvature)
                            double dot_product = direction[0] * prev_direction[0] + direction[1] * prev_direction[1] + direction[2] * prev_direction[2];
                            dot_product = std::clamp(dot_product, -1.0, 1.0);
                            double angle_change = std::acos(dot_product);

                            // Curvature approximation: angle change / average segment length
                            double avg_length = (segment_length + prev_length) * 0.5;
                            curvature = angle_change / std::max(avg_length, 1e-6);
                        }
                    }

                    // Calculate time-optimal velocity for each axis
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        if (segment_length > 1e-6)
                        {
                            // Base velocity from direction and max velocity
                            double direction_component = direction[axis];

                            // Start with maximum possible velocity for this axis
                            double max_vel_for_axis = maxVel[axis];

                            // Reduce velocity based on curvature (centripetal acceleration constraint)
                            // v = sqrt(a_max / curvature) for safe cornering
                            if (curvature > 1e-6)
                            {
                                double curvature_limited_vel = std::sqrt(maxAcc[axis] / curvature);
                                max_vel_for_axis = std::min(max_vel_for_axis, curvature_limited_vel);
                            }

                            // Consider upcoming corner if this isn't the last segment
                            if (wp_idx + 2 < waypoints.size())
                            {
                                const auto &next_next_waypoint = waypoints[wp_idx + 2];
                                double next_direction[3] = {
                                    next_next_waypoint.x - next_waypoint.x,
                                    next_next_waypoint.y - next_waypoint.y,
                                    next_next_waypoint.r - next_waypoint.r};

                                double next_length = std::sqrt(next_direction[0] * next_direction[0] + next_direction[1] * next_direction[1] + next_direction[2] * next_direction[2]);

                                if (next_length > 1e-6)
                                {
                                    // Normalize
                                    for (int i = 0; i < 3; ++i)
                                        next_direction[i] /= next_length;

                                    // Calculate next corner curvature
                                    double next_dot = direction[0] * next_direction[0] + direction[1] * next_direction[1] + direction[2] * next_direction[2];
                                    next_dot = std::clamp(next_dot, -1.0, 1.0);
                                    double next_angle = std::acos(next_dot);
                                    double next_curvature = next_angle / std::max((segment_length + next_length) * 0.5, 1e-6);

                                    // Limit velocity to allow deceleration for the next corner
                                    if (next_curvature > 1e-6)
                                    {
                                        double next_corner_vel = std::sqrt(maxAcc[axis] / next_curvature);
                                        // Calculate required deceleration distance
                                        double decel_distance = (max_vel_for_axis * max_vel_for_axis - next_corner_vel * next_corner_vel) / (2.0 * maxAcc[axis]);

                                        if (decel_distance > segment_length * 0.5)
                                        {
                                            // Need to reduce velocity to allow proper deceleration
                                            double adjusted_vel = std::sqrt(next_corner_vel * next_corner_vel + 2.0 * maxAcc[axis] * segment_length * 0.5);
                                            max_vel_for_axis = std::min(max_vel_for_axis, adjusted_vel);
                                        }
                                    }
                                }
                            }

                            // Apply direction component and ensure we don't exceed limits
                            target_vel[axis] = direction_component * max_vel_for_axis;
                            target_vel[axis] = std::clamp(target_vel[axis], -maxVel[axis], maxVel[axis]);
                        }
                        else
                        {
                            target_vel[axis] = 0.0;
                        }

                        // For time-optimal trajectories, allow non-zero acceleration at waypoints
                        // We'll compute this based on the direction change and curvature
                        if (wp_idx > 0)
                        {
                            // Look at previous direction to compute acceleration for smooth cornering
                            const auto &prev_waypoint = waypoints[wp_idx - 1];
                            double prev_direction[3] = {
                                waypoint.x - prev_waypoint.x,
                                waypoint.y - prev_waypoint.y,
                                waypoint.r - prev_waypoint.r};

                            double prev_mag = std::sqrt(prev_direction[0] * prev_direction[0] + prev_direction[1] * prev_direction[1] + prev_direction[2] * prev_direction[2]);

                            if (prev_mag > 1e-6 && segment_length > 1e-6)
                            {
                                // Compute direction change (curvature approximation)
                                double direction_change = (direction[axis] / segment_length) - (prev_direction[axis] / prev_mag);
                                // Scale acceleration based on direction change and max acceleration
                                target_acc[axis] = direction_change * maxAcc[axis] * 0.3; // Use 30% of max acceleration
                                // Clamp to reasonable bounds
                                target_acc[axis] = std::clamp(target_acc[axis], -maxAcc[axis] * 0.5, maxAcc[axis] * 0.5);
                            }
                            else
                            {
                                target_acc[axis] = 0.0;
                            }
                        }
                        else
                        {
                            // First waypoint - consider starting state as "previous" state
                            double start_to_waypoint[3] = {
                                waypoint.x - current_pos[axis],
                                waypoint.y - current_pos[axis == 0 ? 0 : (axis == 1 ? 1 : 2)],
                                waypoint.r - current_pos[axis == 0 ? 0 : (axis == 1 ? 1 : 2)]};

                            // Correct the indexing for start_to_waypoint
                            start_to_waypoint[0] = waypoint.x - current_pos[0];
                            start_to_waypoint[1] = waypoint.y - current_pos[1];
                            start_to_waypoint[2] = waypoint.r - current_pos[2];

                            double start_mag = std::sqrt(start_to_waypoint[0] * start_to_waypoint[0] +
                                                         start_to_waypoint[1] * start_to_waypoint[1] +
                                                         start_to_waypoint[2] * start_to_waypoint[2]);

                            if (start_mag > 1e-6 && segment_length > 1e-6)
                            {
                                // Compute direction change from start to first waypoint to next waypoint
                                double start_direction = start_to_waypoint[axis] / start_mag;
                                double next_direction = direction[axis] / segment_length;
                                double direction_change = next_direction - start_direction;

                                // For first waypoint, also consider the starting acceleration
                                // Blend between continuing the starting acceleration and the path-based acceleration
                                double path_acc = direction_change * maxAcc[axis] * 0.3;
                                double blend_factor = 0.7; // 70% path-based, 30% continuation of start acceleration
                                target_acc[axis] = blend_factor * path_acc + (1.0 - blend_factor) * current_acc[axis];

                                // Clamp to reasonable bounds
                                target_acc[axis] = std::clamp(target_acc[axis], -maxAcc[axis] * 0.5, maxAcc[axis] * 0.5);
                            }
                            else
                            {
                                // If no clear direction, use a blend of starting acceleration and small path correction
                                target_acc[axis] = current_acc[axis] * 0.5; // 50% of starting acceleration
                                target_acc[axis] = std::clamp(target_acc[axis], -maxAcc[axis] * 0.3, maxAcc[axis] * 0.3);
                            }
                        }
                    }
                }

                // Find time-optimal duration for this segment using binary search
                double min_time = 0.001; // Minimum reasonable time
                double max_time = 10.0;  // Maximum reasonable time
                double optimal_time = min_time;

                for (int iter = 0; iter < static_cast<int>(maxIterations); ++iter)
                {
                    double test_time = (min_time + max_time) * 0.5;

                    bool feasible = true;
                    double test_coeffs[3][4];

                    // Compute cubic polynomial coefficients for each axis
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        double p0 = current_pos[axis];
                        double v0 = current_vel[axis];
                        double pf = target_pos[axis];
                        double vf = target_vel[axis];
                        double T = test_time;
                        double T2 = T * T;
                        double T3 = T2 * T;

                        // Cubic polynomial: p(t) = a0 + a1*t + a2*t^2 + a3*t^3
                        // Constraints: p(0) = p0, p'(0) = v0, p(T) = pf, p'(T) = vf
                        test_coeffs[axis][0] = p0;
                        test_coeffs[axis][1] = v0;
                        test_coeffs[axis][2] = (3 * (pf - p0) - T * (2 * v0 + vf)) / T2;
                        test_coeffs[axis][3] = (2 * (p0 - pf) + T * (v0 + vf)) / T3;

                        // Check velocity and acceleration constraints along the trajectory
                        const int num_checks = 50;
                        for (int check = 0; check <= num_checks && feasible; ++check)
                        {
                            double t = (check * T) / num_checks;
                            double t2 = t * t;

                            // Compute velocity: v(t) = a1 + 2*a2*t + 3*a3*t^2
                            double vel = test_coeffs[axis][1] + 2 * test_coeffs[axis][2] * t + 3 * test_coeffs[axis][3] * t2;

                            // Compute acceleration: a(t) = 2*a2 + 6*a3*t
                            double acc = 2 * test_coeffs[axis][2] + 6 * test_coeffs[axis][3] * t;

                            // Check constraints
                            if (std::abs(vel) > maxVel[axis] * 1.01 || std::abs(acc) > maxAcc[axis] * 1.01)
                            {
                                feasible = false;
                            }
                        }
                    }

                    if (feasible)
                    {
                        optimal_time = test_time;
                        max_time = test_time;
                        // Note: Don't store coefficients here, we'll create the final segment after optimization
                    }
                    else
                    {
                        min_time = test_time;
                    }

                    if (max_time - min_time < 1e-6)
                        break;
                }

                // Create and store the segment for this waypoint
                TrajectorySegment segment;
                segment.duration = optimal_time;

                // Store start conditions
                for (int axis = 0; axis < 3; ++axis)
                {
                    segment.start_pos[axis] = current_pos[axis];
                    segment.start_vel[axis] = current_vel[axis];
                    segment.start_acc[axis] = current_acc[axis];
                    segment.end_pos[axis] = target_pos[axis];
                    segment.end_vel[axis] = target_vel[axis];
                    segment.end_acc[axis] = target_acc[axis];
                }

                // Compute final coefficients with optimal time
                for (int axis = 0; axis < 3; ++axis)
                {
                    double p0 = current_pos[axis];
                    double v0 = current_vel[axis];
                    double pf = target_pos[axis];
                    double vf = target_vel[axis];
                    double T = optimal_time;
                    double T2 = T * T;
                    double T3 = T2 * T;

                    segment.coeffs[axis][0] = p0;
                    segment.coeffs[axis][1] = v0;
                    segment.coeffs[axis][2] = (3 * (pf - p0) - T * (2 * v0 + vf)) / T2;
                    segment.coeffs[axis][3] = (2 * (p0 - pf) + T * (v0 + vf)) / T3;
                }

                // Add this segment to our trajectory
                segments.push_back(segment);

                // Extract actual velocity and acceleration from the polynomial at the endpoint (t = optimal_time)
                double actual_vel[3], actual_acc[3];
                for (int axis = 0; axis < 3; ++axis)
                {
                    double T = optimal_time;
                    double T2 = T * T;

                    // Get the final segment for this waypoint
                    const auto &final_segment = segments.back();

                    // Velocity: v(t) = a1 + 2*a2*t + 3*a3*t^2
                    actual_vel[axis] = final_segment.coeffs[axis][1] +
                                       2 * final_segment.coeffs[axis][2] * T +
                                       3 * final_segment.coeffs[axis][3] * T2;

                    // Acceleration: a(t) = 2*a2 + 6*a3*t
                    actual_acc[axis] = 2 * final_segment.coeffs[axis][2] +
                                       6 * final_segment.coeffs[axis][3] * T;
                }

                // Update current state to end of this segment using polynomial results
                for (int axis = 0; axis < 3; ++axis)
                {
                    current_pos[axis] = target_pos[axis];
                    current_vel[axis] = actual_vel[axis];
                    current_acc[axis] = actual_acc[axis];
                }

                // Set waypoint velocity and acceleration from the computed polynomial trajectory
                spdlog::info("Waypoint {} polynomial vel: ({:.3f}, {:.3f}, {:.3f}), acc: ({:.3f}, {:.3f}, {:.3f})",
                             wp_idx, actual_vel[0], actual_vel[1], actual_vel[2],
                             actual_acc[0], actual_acc[1], actual_acc[2]);
                waypoint.vx = actual_vel[0] * 0.5;
                waypoint.vy = actual_vel[1] * 0.5;
                waypoint.vr = actual_vel[2] * 0.5;
                waypoint.ax = actual_acc[0] * 0.5;
                waypoint.ay = actual_acc[1] * 0.5;
                waypoint.ar = 0;
                // actual_acc[2] * 0.5;
            }

            // Log summary information about the computed trajectory
            double total_time = 0.0;
            for (const auto &segment : segments)
            {
                total_time += segment.duration;
            }

            spdlog::info("Computed time-optimal trajectory with {} segments, total duration: {:.3f}s",
                         segments.size(), total_time);
        }
    };
}