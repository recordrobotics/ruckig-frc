module;

#include <spdlog/spdlog.h>
#include <imgui.h>
#include <ruckig/ruckig.hpp>
#include <math.h>
#include <algorithm>
#include <cfloat>

export module ui.sources.ruckig_local;

import ui.sources.ruckigsource;
import ui.uimodule;
import robot;
import pid;
import random;

using namespace ruckig;

export namespace ui
{
    namespace sources
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

        class RuckigLocal : public RuckigSource
        {
        public:
            RuckigLocal() : ruckig(0.02), robot()
            {
                next_delta_time = ruckig.delta_time;
            }

            void init() override
            {
                input.duration_discretization = DurationDiscretization::Discrete;

                input.per_dof_synchronization = {Synchronization::Phase, Synchronization::Phase, Synchronization::None};

                input.max_velocity = {4.0, 4.0, 4.0};
                input.max_acceleration = {10.0, 10.0, 10.0};
                input.max_jerk = {20.0, 20.0, 20.0};

                input.target_position = {0.0, 0.0, 0.0};

                output.calculation_duration = 0;

                waypoints.clear();
            }

            void deinit() override
            {
            }

            void fullReset() override
            {
                ruckig.reset();

                ruckig.delta_time = 0.02; // Reset delta time
                next_delta_time = ruckig.delta_time;
                input = InputParameter<3>();
                output = OutputParameter<3>();
                init();
            }

            void reset() override
            {
                robot.setPosition(0.0, 0.0, 0.0);
                input.current_position = {0.0, 0.0, 0.0};
                input.current_velocity = {0.0, 0.0, 0.0};
                input.current_acceleration = {0.0, 0.0, 0.0};
                position_pid_x.reset();
                position_pid_y.reset();
                position_pid_r.reset();

                fitWaypoints();
            }

            void update() override
            {
                last_delta_time = next_delta_time;

                double current_time = ImGui::GetTime();
                if (next_delta_time > 0.0 && (current_time - last_time) >= next_delta_time)
                {
                    last_time = current_time;
                    updateStep();
                    next_delta_time = ruckig.delta_time +
                                      (rnd::random_double(0, 1) < loop_overrun_frequency ? rnd::random_double(0, loop_overrun_amount) : 0); // Simulate loop overrun
                }
            }

            void drawMenuBar() override
            {
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
            }

            void drawGUI() override
            {
                if (ImGui::CollapsingHeader("Configuration", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    ImGui::Text("Degrees of Freedom: %zu", input.degrees_of_freedom);

                    double dt = ruckig.delta_time;
                    double min_delta_time = 0.001, max_delta_time = 1.0;
                    ImGui::TextUnformatted("Delta Time");
                    ImGui::SameLine();
                    ImGui::SliderScalar("##delta_time", ImGuiDataType_Double, &dt, &min_delta_time, &max_delta_time, "%.3f");

                    if (ruckig.delta_time != dt)
                    {
                        ruckig.delta_time = dt;
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

            std::array<double, 3> getActualPosition() const override
            {
                double dx, dy, dr;
                robot.getState(&dx, &dy, &dr);
                return {dx, dy, dr};
            }

            std::array<double, 3> getActualVelocity() const override
            {
                double vx, vy, vr;
                robot.getState(nullptr, nullptr, nullptr, &vx, &vy, &vr);
                return {vx, vy, vr};
            }
            std::array<double, 3> getActualAcceleration() const override
            {
                double ax, ay, ar;
                robot.getState(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &ax, &ay, &ar);
                return {ax, ay, ar};
            }
            std::array<double, 3> getSetpointPosition() const override
            {
                return {input.current_position[0], input.current_position[1], input.current_position[2]};
            }
            std::array<double, 3> getSetpointVelocity() const override
            {
                return {input.current_velocity[0], input.current_velocity[1], input.current_velocity[2]};
            }
            std::array<double, 3> getSetpointAcceleration() const override
            {
                return {input.current_acceleration[0], input.current_acceleration[1], input.current_acceleration[2]};
            }
            std::array<double, 3> getTargetPosition() const override
            {
                return {input.target_position[0], input.target_position[1], input.target_position[2]};
            }
            std::array<double, 3> getTargetVelocity() const override
            {
                return {input.target_velocity[0], input.target_velocity[1], input.target_velocity[2]};
            }
            std::array<double, 3> getTargetAcceleration() const override
            {
                return {input.target_acceleration[0], input.target_acceleration[1], input.target_acceleration[2]};
            }
            std::array<double, 3> getNewPosition() const override
            {
                return {output.new_position[0], output.new_position[1], output.new_position[2]};
            }
            std::array<double, 3> getNewVelocity() const override
            {
                return {output.new_velocity[0], output.new_velocity[1], output.new_velocity[2]};
            }
            std::array<double, 3> getNewAcceleration() const override
            {
                return {output.new_acceleration[0], output.new_acceleration[1], output.new_acceleration[2]};
            }
            std::array<double, 3> getNewJerk() const override
            {
                return {output.new_jerk[0], output.new_jerk[1], output.new_jerk[2]};
            }
            std::array<double, 3> getMaxVelocity() const override
            {
                return {input.max_velocity[0], input.max_velocity[1], input.max_velocity[2]};
            }
            std::array<double, 3> getMaxAcceleration() const override
            {
                return {input.max_acceleration[0], input.max_acceleration[1], input.max_acceleration[2]};
            }
            std::array<double, 3> getMaxJerk() const override
            {
                return {input.max_jerk[0], input.max_jerk[1], input.max_jerk[2]};
            }

            std::array<ruckig::Synchronization, 3> getPerDoFSynchronization() const override
            {
                return {input.per_dof_synchronization.value()[0], input.per_dof_synchronization.value()[1], input.per_dof_synchronization.value()[2]};
            }
            ruckig::Synchronization getSynchronization() const override
            {
                return input.synchronization;
            }
            ruckig::DurationDiscretization getDurationDiscretization() const override
            {
                return input.duration_discretization;
            }

            std::vector<Pose2d> getWaypoints() override
            {
                return waypoints;
            }
            void addWaypoint() override
            {
                addWaypoint(input.current_position[0], input.current_position[1], input.current_position[2]);
            }
            void removeWaypoint(int index) override
            {
                if (index >= 0 && index < waypoints.size())
                {
                    waypoints.erase(waypoints.begin() + index);
                    if (current_waypoint >= waypoints.size())
                    {
                        current_waypoint = -1; // Reset to start if current waypoint is out of bounds
                    }
                    fitWaypoints();
                }
            }

            double getDeltaTime() const override
            {
                return ruckig.delta_time;
            }
            double getActualDeltaTime() const override
            {
                return next_delta_time;
            }

            void updateWaypoints(float x, float y) override
            {
                // Set waypoint position for all editing waypoints
                for (auto &wp : waypoints)
                {
                    if (wp.editing)
                    {
                        wp.x = x;
                        wp.y = y;
                        fitWaypoints();
                    }
                }
            }

            void editWaypoint(int index, Pose2d wp) override
            {
                if (index >= 0 && index < waypoints.size())
                {
                    waypoints[index] = wp;
                }
            }

            bool canEditWaypoints() const override
            {
                return true;
            }

            bool isMonotonic() const override
            {
                return false;
            }

            std::string getTitle() const override
            {
                return "Ruckig FRC - Local";
            }

            double getRobotSize() const override {
                return 0.9;
            }

            double getCalculationTime() const override {
                return output.calculation_duration;
            }

            ruckig::Result getResult() const override {
                return ruckigResult;
            }

        private:
            Ruckig<3>
                ruckig;
            InputParameter<3> input;
            OutputParameter<3> output;
            Robot robot;
            RobotControlMode control_mode = RobotControlMode::Combined;
            WaypointMode waypoint_mode = WaypointMode::LinearAcceleration;

            double last_time = 0.0;

            double position_pid_kp = 46.67;
            double position_pid_ki = 0;
            double position_pid_kd = 0.22;
            double rotation_pid_kp = 51.48;
            double rotation_pid_ki = 0;
            double rotation_pid_kd = 0.05;
            PID position_pid_x;
            PID position_pid_y;
            PID position_pid_r;

            double last_delta_time = 0.02;
            double next_delta_time = 0.02;

            double loop_overrun_frequency = 0.0;
            double loop_overrun_amount = 0.01;
            double position_noise_amount = 0.1;
            double rotation_noise_amount = 0.1;

            std::vector<Pose2d> waypoints;
            int current_waypoint = -1;

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

                    ruckigResult = ruckig.update(input, output);
                    output.pass_to_input(input);

                    double x = output.new_position[0];
                    double y = output.new_position[1];
                    double r = output.new_position[2];
                    double vx = 0, vy = 0, vr = 0;

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

                    if (updateCallback != nullptr)
                    {
                        updateCallback();
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

            inline void fitWaypoints()
            {
                const int max_iterations = 10000; // Maximum iterations for waypoint calculation

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

            inline float rgbToHue(const ImColor &color)
            {
                float h, s, v;
                ImGui::ColorConvertRGBtoHSV(color.Value.x, color.Value.y, color.Value.z, h, s, v);
                return h;
            }
        };
    };
};