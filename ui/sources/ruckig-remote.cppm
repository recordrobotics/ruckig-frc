module;

#include <spdlog/spdlog.h>
#include <imgui.h>
#include <ruckig/ruckig.hpp>
#include <math.h>
#include <algorithm>
#include <cfloat>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

export module ui.sources.ruckig_remote;

import ui.sources.ruckigsource;
import ui.uimodule;
import robot;

using namespace ruckig;
using namespace std::string_view_literals;

export namespace ui
{
    namespace sources
    {
        class RuckigRemote : public RuckigSource
        {
        public:
            RuckigRemote()
            {
            }

            void init() override
            {
                inst = nt::NetworkTableInstance::Create();
                inst.AddConnectionListener(true, [this](const nt::Event &event)
                                           {
                                               if (event.Is(nt::EventFlags::kConnected))
                                               {
                                                    auto connInfo = std::get<nt::ConnectionInfo>(event.data);
                                                    spdlog::info("Connected to NetworkTables server at {}:{}", connInfo.remote_ip, connInfo.remote_port);
                                                    setTitleDirty();
                                               }
                                               else if( event.Is(nt::EventFlags::kDisconnected))
                                               {
                                                    auto connInfo = std::get<nt::ConnectionInfo>(event.data);
                                                    spdlog::warn("Disconnected from NetworkTables server at {}:{}", connInfo.remote_ip, connInfo.remote_port);
                                                    setTitleDirty();
                                               } });
                std::array<std::string_view, 1> topics = {"/Ruckig/"sv};
                inst.AddListener(topics, nt::EventFlags::kValueAll, [this](const nt::Event &event)
                                 {
                                    std::string prefix = "/Ruckig/" + telemetryName + "/";

                                    const auto data = event.GetValueEventData();
                                    const auto &name = nt::GetTopicName(data->topic);
                                    const auto &value = data->value;

                                    gotUpdate = true;

                                    std::scoped_lock lock(dataMutex);
                                    if(name.ends_with("/Active")) {
                                        std::string actualTelemetryName = name.substr(8, name.length() - 15); // Remove "/Ruckig/" and "/Active"
                                        if(value.GetBoolean()) {
                                            spdlog::info("New telemetry instance '{}'", actualTelemetryName);

                                            if(telemetryNames.empty()) {
                                                telemetryName = actualTelemetryName;
                                                updateValuesWithGet();
                                            }

                                            if(std::find(telemetryNames.begin(), telemetryNames.end(), actualTelemetryName) == telemetryNames.end()) {
                                                telemetryNames.push_back(actualTelemetryName);
                                            }
                                        } else {
                                            spdlog::info("Lost telemetry instance '{}'", actualTelemetryName);
                                            telemetryNames.erase(std::remove(telemetryNames.begin(), telemetryNames.end(), actualTelemetryName), telemetryNames.end());
                                            if(telemetryName == actualTelemetryName) {
                                                if(!telemetryNames.empty()) {
                                                    telemetryName = telemetryNames.front();
                                                    updateValuesWithGet();
                                                } else {
                                                    telemetryName = "";
                                                }
                                            }
                                        }
                                    }
                                    else if (name == prefix + "ActualPosition")
                                    {
                                    actualPosition.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "ActualVelocity")
                                    {
                                    actualVelocity.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "SetpointPosition")
                                    {
                                    setpointPosition.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "SetpointVelocity")
                                    {
                                    setpointVelocity.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "SetpointAcceleration")
                                    {
                                    setpointAcceleration.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "TargetPosition")
                                    {
                                    targetPosition.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "TargetVelocity")
                                    {
                                    targetVelocity.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "TargetAcceleration")
                                    {
                                    targetAcceleration.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "NewPosition")
                                    {
                                    newPosition.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "NewVelocity")
                                    {
                                    newVelocity.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "NewAcceleration")
                                    {
                                    newAcceleration.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "NewJerk")
                                    {
                                    newJerk.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "MaxVelocity")
                                    {
                                    maxVelocity.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    } 
                                    else if (name == prefix + "MaxAcceleration")
                                    {
                                    maxAcceleration.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "MaxJerk")
                                    {
                                    maxJerk.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    }
                                    else if (name == prefix + "PerDoFSynchronization")
                                    {
                                        auto syncInts = value.GetIntegerArray();
                                        perDoFSynchronization.clear();
                                        for (int64_t v : syncInts)
                                        {
                                            perDoFSynchronization.push_back(static_cast<Synchronization>(v));
                                        }
                                    }
                                    else if (name == prefix + "DurationDiscretization")
                                    {
                                        durationDiscretization = static_cast<DurationDiscretization>(value.GetInteger());
                                    } 
                                    else if (name == prefix + "Synchronization")
                                    {
                                        synchronization = static_cast<Synchronization>(value.GetInteger());
                                    } else if(name == prefix + "Result") {
                                        lastResult = static_cast<Result>(value.GetInteger());
                                    } else if(name == prefix + "Waypoints/DOF") {
                                        waypoint_dof = static_cast<int>(value.GetInteger());
                                    } else if(name == prefix + "Waypoints/Position") {
                                        waypoint_positions.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    } else if(name == prefix + "Waypoints/Velocity") {
                                        waypoint_velocities.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    } else if(name == prefix + "Waypoints/Acceleration") {
                                        waypoint_accelerations.assign(value.GetDoubleArray().begin(), value.GetDoubleArray().end());
                                    } else if(name == prefix + "RobotSize") {
                                        robotSize = value.GetDouble();
                                    } else if(name == prefix + "CalculationDuration") {
                                        calculationDuration = value.GetDouble();
                                    } });
            }

            void deinit() override
            {
                nt::NetworkTableInstance::Destroy(inst);
            }

            void fullReset() override
            {
            }

            void reset() override
            {
            }

            void update() override
            {
                {
                    waypoints.clear();
                    std::scoped_lock lock(dataMutex);
                    if (waypoint_dof >= 3)
                    {
                        for (size_t i = 0; i < waypoint_positions.size(); i += waypoint_dof)
                        {
                            if (waypoint_velocities.size() <= i || waypoint_accelerations.size() <= i)
                            {
                                break;
                            }

                            Pose2d wp;
                            wp.x = waypoint_positions[i];
                            wp.y = waypoint_positions[i + 1];
                            wp.r = waypoint_positions[i + 2];
                            wp.vx = waypoint_velocities[i];
                            wp.vy = waypoint_velocities[i + 1];
                            wp.vr = waypoint_velocities[i + 2];
                            wp.ax = waypoint_accelerations[i];
                            wp.ay = waypoint_accelerations[i + 1];
                            wp.ar = waypoint_accelerations[i + 2];
                            float hue = waypoints.empty() ? 0.2f : (rgbToHue(waypoints.back().color) + 0.3f);
                            wp.editing = false;
                            wp.color = ImColor::HSV(hue, 1.0f, 1.0f);
                            waypoints.push_back(wp);
                        }
                    }
                }

                double positionDiff = 0.0;
                for (size_t i = 0; i < std::min(actualPosition.size(), previousActualPosition.size()); i++)
                {
                    double dx = actualPosition[i] - previousActualPosition[i];
                    positionDiff += dx * dx;
                }

                positionDiff = std::sqrt(positionDiff);
                previousActualPosition = actualPosition;

                if (previousResult == Result::Finished && lastResult != Result::Finished || positionDiff > 0.1)
                {
                    if (resetRobotTrigger != nullptr)
                    {
                        resetRobotTrigger();
                    }
                }
                previousResult = lastResult;

                if (gotUpdate && updateCallback != nullptr)
                {
                    gotUpdate = false;
                    updateCallback();
                }
            }

            void drawMenuBar() override
            {
                if (ImGui::BeginMenu("Remote"))
                {
                    bool robotSelected = isConnecting && !isSimulation;
                    if (ImGui::MenuItem("Connect Robot", nullptr, &robotSelected))
                    {
                        if (inst && inst.IsConnected() && isSimulation)
                        {
                            inst.StopClient();
                        }
                        inst.SetServerTeam(teamNumber);
                        inst.StartClient4("RuckigRemote");
                        isSimulation = false;
                        isConnecting = true;
                        setTitleDirty();
                    }

                    bool simSelected = isConnecting && isSimulation;
                    if (ImGui::MenuItem("Connect Simulator", nullptr, &simSelected))
                    {
                        if (inst && inst.IsConnected() && !isSimulation)
                        {
                            inst.StopClient();
                        }
                        inst.SetServer("127.0.0.1");
                        inst.StartClient4("RuckigRemote");
                        isSimulation = true;
                        isConnecting = true;
                        setTitleDirty();
                    }

                    ImGui::EndMenu();
                }
            }

            void drawGUI() override
            {
                if (ImGui::CollapsingHeader("Configuration", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    ImGui::Text("Degrees of Freedom: %zu", actualPosition.size());

                    ImGui::Text("Team Number");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::InputInt("##team_number", &teamNumber, 0, 0))
                    {
                        teamNumber = std::clamp(teamNumber, 0, 9999);
                        if (inst && inst.IsConnected() && !isSimulation)
                        {
                            inst.StopClient();
                            inst.SetServerTeam(teamNumber);
                            inst.StartClient4("RuckigRemote");
                            setTitleDirty();
                        }
                        else if (inst && !isSimulation)
                        {
                            inst.SetServerTeam(teamNumber);
                            setTitleDirty();
                        }
                    }

                    ImGui::Spacing();

                    ImGui::Text("Ruckig Instance");
                    if (ImGui::BeginCombo("##telemetry_name", telemetryName.c_str()))
                    {
                        for (const auto &name : telemetryNames)
                        {
                            bool is_selected = (telemetryName == name);
                            if (ImGui::Selectable(name.c_str(), is_selected))
                            {
                                telemetryName = name;
                            }
                            if (is_selected)
                            {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }
                }

                ImGui::Spacing();

                if (ImGui::CollapsingHeader("Input Parameters", ImGuiTreeNodeFlags_DefaultOpen))
                {
                    if (!maxVelocity.empty())
                    {
                        double min_vel = 0.01, max_vel = 4.0;
                        ImGui::Text("Max Velocity");
                        ImGui::BeginDisabled();
                        ImGui::SliderScalarN("##max_velocity", ImGuiDataType_Double, maxVelocity.data(), static_cast<int>(maxVelocity.size()), &min_vel, &max_vel, "%.2f");
                        ImGui::EndDisabled();
                        ImGui::Spacing();
                    }

                    if (!maxAcceleration.empty())
                    {
                        double min_acc = 0.01, max_acc = 10.0;
                        ImGui::Text("Max Acceleration");
                        ImGui::BeginDisabled();
                        ImGui::SliderScalarN("##max_acceleration", ImGuiDataType_Double, maxAcceleration.data(), static_cast<int>(maxAcceleration.size()), &min_acc, &max_acc, "%.2f");
                        ImGui::EndDisabled();
                        ImGui::Spacing();
                    }

                    if (!maxJerk.empty())
                    {
                        double min_jerk = 0.01, max_jerk = 20.0;
                        ImGui::Text("Max Jerk");
                        ImGui::BeginDisabled();
                        ImGui::SliderScalarN("##max_jerk", ImGuiDataType_Double, maxJerk.data(), static_cast<int>(maxJerk.size()), &min_jerk, &max_jerk, "%.2f");
                        ImGui::EndDisabled();
                        ImGui::Spacing();
                    }

                    // Dropdown for input.synchronization options
                    static const char *sync_options[] = {"Time", "TimeIfNecessary", "Phase", "None"};

                    ImGui::TextUnformatted("Synchronization (per DoF)");
                    float total_width = ImGui::GetContentRegionAvail().x;
                    float combo_width = total_width / 3.0f - 14.0f; // Subtract a few pixels for spacing
                    ImGui::BeginDisabled();
                    for (int i = 0; i < perDoFSynchronization.size(); ++i)
                    {
                        if (i > 0)
                            ImGui::SameLine();
                        int current_sync = static_cast<int>(perDoFSynchronization[i]);
                        std::string combo_label = "##synchronization_" + std::to_string(i);
                        ImGui::SetNextItemWidth(combo_width);
                        if (ImGui::BeginCombo(combo_label.c_str(), sync_options[current_sync]))
                        {
                            for (int n = 0; n < IM_ARRAYSIZE(sync_options); n++)
                            {
                                bool is_selected = (current_sync == n);
                                if (ImGui::Selectable(sync_options[n], is_selected))
                                {
                                    perDoFSynchronization[i] = static_cast<Synchronization>(n);
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
                    ImGui::EndDisabled();

                    ImGui::Spacing();

                    // Dropdown for input.duration_discretization options
                    static const char *duration_discretization_options[] = {"Continuous", "Discrete"};
                    int current_duration_discretization = static_cast<int>(durationDiscretization);

                    ImGui::TextUnformatted("Duration Discretization");
                    ImGui::BeginDisabled();
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

                        durationDiscretization = static_cast<DurationDiscretization>(current_duration_discretization);
                    }
                    ImGui::EndDisabled();
                }
            }

            std::array<double, 3> getActualPosition() const override
            {
                return {
                    actualPosition.size() > 0 ? actualPosition[0] : 0,
                    actualPosition.size() > 1 ? actualPosition[1] : 0,
                    actualPosition.size() > 2 ? actualPosition[2] : 0};
            }

            std::array<double, 3> getActualVelocity() const override
            {
                return {
                    actualVelocity.size() > 0 ? actualVelocity[0] : 0,
                    actualVelocity.size() > 1 ? actualVelocity[1] : 0,
                    actualVelocity.size() > 2 ? actualVelocity[2] : 0};
            }
            std::array<double, 3> getActualAcceleration() const override
            {
                return {0, 0, 0};
            }
            std::array<double, 3> getSetpointPosition() const override
            {
                return {
                    setpointPosition.size() > 0 ? setpointPosition[0] : 0,
                    setpointPosition.size() > 1 ? setpointPosition[1] : 0,
                    setpointPosition.size() > 2 ? setpointPosition[2] : 0};
            }
            std::array<double, 3> getSetpointVelocity() const override
            {
                return {
                    setpointVelocity.size() > 0 ? setpointVelocity[0] : 0,
                    setpointVelocity.size() > 1 ? setpointVelocity[1] : 0,
                    setpointVelocity.size() > 2 ? setpointVelocity[2] : 0};
            }
            std::array<double, 3> getSetpointAcceleration() const override
            {
                return {
                    setpointAcceleration.size() > 0 ? setpointAcceleration[0] : 0,
                    setpointAcceleration.size() > 1 ? setpointAcceleration[1] : 0,
                    setpointAcceleration.size() > 2 ? setpointAcceleration[2] : 0};
            }
            std::array<double, 3> getTargetPosition() const override
            {
                return {
                    targetPosition.size() > 0 ? targetPosition[0] : 0,
                    targetPosition.size() > 1 ? targetPosition[1] : 0,
                    targetPosition.size() > 2 ? targetPosition[2] : 0};
            }
            std::array<double, 3> getTargetVelocity() const override
            {
                return {
                    targetVelocity.size() > 0 ? targetVelocity[0] : 0,
                    targetVelocity.size() > 1 ? targetVelocity[1] : 0,
                    targetVelocity.size() > 2 ? targetVelocity[2] : 0};
            }
            std::array<double, 3> getTargetAcceleration() const override
            {
                return {
                    targetAcceleration.size() > 0 ? targetAcceleration[0] : 0,
                    targetAcceleration.size() > 1 ? targetAcceleration[1] : 0,
                    targetAcceleration.size() > 2 ? targetAcceleration[2] : 0};
            }
            std::array<double, 3> getNewPosition() const override
            {
                return {
                    newPosition.size() > 0 ? newPosition[0] : 0,
                    newPosition.size() > 1 ? newPosition[1] : 0,
                    newPosition.size() > 2 ? newPosition[2] : 0};
            }
            std::array<double, 3> getNewVelocity() const override
            {
                return {
                    newVelocity.size() > 0 ? newVelocity[0] : 0,
                    newVelocity.size() > 1 ? newVelocity[1] : 0,
                    newVelocity.size() > 2 ? newVelocity[2] : 0};
            }
            std::array<double, 3> getNewAcceleration() const override
            {
                return {
                    newAcceleration.size() > 0 ? newAcceleration[0] : 0,
                    newAcceleration.size() > 1 ? newAcceleration[1] : 0,
                    newAcceleration.size() > 2 ? newAcceleration[2] : 0};
            }
            std::array<double, 3> getNewJerk() const override
            {
                return {
                    newJerk.size() > 0 ? newJerk[0] : 0,
                    newJerk.size() > 1 ? newJerk[1] : 0,
                    newJerk.size() > 2 ? newJerk[2] : 0};
            }
            std::array<double, 3> getMaxVelocity() const override
            {
                return {
                    maxVelocity.size() > 0 ? maxVelocity[0] : 0,
                    maxVelocity.size() > 1 ? maxVelocity[1] : 0,
                    maxVelocity.size() > 2 ? maxVelocity[2] : 0};
            }
            std::array<double, 3> getMaxAcceleration() const override
            {
                return {
                    maxAcceleration.size() > 0 ? maxAcceleration[0] : 0,
                    maxAcceleration.size() > 1 ? maxAcceleration[1] : 0,
                    maxAcceleration.size() > 2 ? maxAcceleration[2] : 0};
            }
            std::array<double, 3> getMaxJerk() const override
            {
                return {
                    maxJerk.size() > 0 ? maxJerk[0] : 0,
                    maxJerk.size() > 1 ? maxJerk[1] : 0,
                    maxJerk.size() > 2 ? maxJerk[2] : 0};
            }

            std::array<ruckig::Synchronization, 3> getPerDoFSynchronization() const override
            {
                return {
                    perDoFSynchronization.size() > 0 ? perDoFSynchronization[0] : ruckig::Synchronization::Time,
                    perDoFSynchronization.size() > 1 ? perDoFSynchronization[1] : ruckig::Synchronization::Time,
                    perDoFSynchronization.size() > 2 ? perDoFSynchronization[2] : ruckig::Synchronization::Time};
            }
            ruckig::Synchronization getSynchronization() const override
            {
                return synchronization;
            }
            ruckig::DurationDiscretization getDurationDiscretization() const override
            {
                return durationDiscretization;
            }

            std::vector<Pose2d> getWaypoints() override
            {
                return waypoints;
            }

            void addWaypoint() override
            {
            }

            void removeWaypoint(int index) override
            {
            }

            double getDeltaTime() const override
            {
                return 0.02;
            }
            double getActualDeltaTime() const override
            {
                return 0.02;
            }

            void updateWaypoints(float x, float y) override
            {
            }

            void editWaypoint(int index, Pose2d wp) override
            {
            }

            bool canEditWaypoints() const override
            {
                return false;
            }

            bool isMonotonic() const override
            {
                return true;
            }

            std::string getTitle() const override
            {
                if (inst && isConnecting)
                {
                    if (inst.IsConnected())
                    {
                        return std::format("Ruckig FRC - Remote (Connected - {}:{})", inst.GetConnections()[0].remote_ip, inst.GetConnections()[0].remote_port);
                    }
                    else
                    {
                        return std::format("Ruckig FRC - Remote (Connecting - {}:{})", isSimulation ? "127.0.0.1" : "Robot " + std::to_string(teamNumber), nt::NetworkTableInstance::kDefaultPort4);
                    }
                }
                else
                {
                    return "Ruckig FRC - Remote (Not Connected)";
                }
            }

            double getRobotSize() const override
            {
                return robotSize;
            }

            double getCalculationTime() const override
            {
                return calculationDuration;
            }

            ruckig::Result getResult() const override
            {
                return lastResult;
            }

        private:
            nt::NetworkTableInstance inst;
            bool isSimulation = false;
            bool isConnecting = false;

            std::vector<double> actualPosition;
            std::vector<double> actualVelocity;
            std::vector<double> setpointPosition;
            std::vector<double> setpointVelocity;
            std::vector<double> setpointAcceleration;
            std::vector<double> targetPosition;
            std::vector<double> targetVelocity;
            std::vector<double> targetAcceleration;
            std::vector<double> newPosition;
            std::vector<double> newVelocity;
            std::vector<double> newAcceleration;
            std::vector<double> newJerk;
            std::vector<double> maxVelocity;
            std::vector<double> maxAcceleration;
            std::vector<double> maxJerk;

            std::vector<ruckig::Synchronization> perDoFSynchronization;
            ruckig::Synchronization synchronization = ruckig::Synchronization::Time;
            ruckig::DurationDiscretization durationDiscretization = ruckig::DurationDiscretization::Continuous;
            double calculationDuration = 0;

            double robotSize = 0.9;

            Result lastResult = Result::Working;
            Result previousResult = Result::Finished;
            std::vector<double> previousActualPosition;
            bool gotUpdate = false;

            int waypoint_dof = 0;
            std::vector<double> waypoint_positions;
            std::vector<double> waypoint_velocities;
            std::vector<double> waypoint_accelerations;

            std::vector<Pose2d> waypoints;

            int teamNumber = 6731;
            std::vector<std::string> telemetryNames;
            std::string telemetryName = "";

            std::mutex dataMutex;

            inline float rgbToHue(const ImColor &color)
            {
                float h, s, v;
                ImGui::ColorConvertRGBtoHSV(color.Value.x, color.Value.y, color.Value.z, h, s, v);
                return h;
            }

            void updateValuesWithGet()
            {
                std::string prefix = "/Ruckig/" + telemetryName + "/";

                auto table = inst.GetTable("Ruckig")->GetSubTable(telemetryName);

                auto actualPosArray = table->GetEntry("ActualPosition").GetDoubleArray({});
                actualPosition.assign(actualPosArray.begin(), actualPosArray.end());

                auto actualVelArray = table->GetEntry("ActualVelocity").GetDoubleArray({});
                actualVelocity.assign(actualVelArray.begin(), actualVelArray.end());

                auto setpointPosArray = table->GetEntry("SetpointPosition").GetDoubleArray({});
                setpointPosition.assign(setpointPosArray.begin(), setpointPosArray.end());

                auto setpointVelArray = table->GetEntry("SetpointVelocity").GetDoubleArray({});
                setpointVelocity.assign(setpointVelArray.begin(), setpointVelArray.end());

                auto setpointAccArray = table->GetEntry("SetpointAcceleration").GetDoubleArray({});
                setpointAcceleration.assign(setpointAccArray.begin(), setpointAccArray.end());

                auto targetPosArray = table->GetEntry("TargetPosition").GetDoubleArray({});
                targetPosition.assign(targetPosArray.begin(), targetPosArray.end());

                auto targetVelArray = table->GetEntry("TargetVelocity").GetDoubleArray({});
                targetVelocity.assign(targetVelArray.begin(), targetVelArray.end());

                auto targetAccArray = table->GetEntry("TargetAcceleration").GetDoubleArray({});
                targetAcceleration.assign(targetAccArray.begin(), targetAccArray.end());

                auto newPosArray = table->GetEntry("NewPosition").GetDoubleArray({});
                newPosition.assign(newPosArray.begin(), newPosArray.end());

                auto newVelArray = table->GetEntry("NewVelocity").GetDoubleArray({});
                newVelocity.assign(newVelArray.begin(), newVelArray.end());

                auto newAccArray = table->GetEntry("NewAcceleration").GetDoubleArray({});
                newAcceleration.assign(newAccArray.begin(), newAccArray.end());

                auto newJerkArray = table->GetEntry("NewJerk").GetDoubleArray({});
                newJerk.assign(newJerkArray.begin(), newJerkArray.end());

                auto maxVelArray = table->GetEntry("MaxVelocity").GetDoubleArray({});
                maxVelocity.assign(maxVelArray.begin(), maxVelArray.end());

                auto maxAccArray = table->GetEntry("MaxAcceleration").GetDoubleArray({});
                maxAcceleration.assign(maxAccArray.begin(), maxAccArray.end());

                auto maxJerkArray = table->GetEntry("MaxJerk").GetDoubleArray({});
                maxJerk.assign(maxJerkArray.begin(), maxJerkArray.end());

                auto syncInts = table->GetEntry("PerDoFSynchronization").GetIntegerArray({});
                perDoFSynchronization.clear();
                for (int64_t v : syncInts)
                {
                    perDoFSynchronization.push_back(static_cast<Synchronization>(v));
                }

                durationDiscretization = static_cast<DurationDiscretization>(table->GetEntry("DurationDiscretization").GetInteger(0));
                synchronization = static_cast<Synchronization>(table->GetEntry("Synchronization").GetInteger(0));
                lastResult = static_cast<Result>(table->GetEntry("Result").GetInteger(0));
                waypoint_dof = static_cast<int>(table->GetEntry("Waypoints/DOF").GetInteger(0));

                auto wpPosArray = table->GetEntry("Waypoints/Position").GetDoubleArray({});
                waypoint_positions.assign(wpPosArray.begin(), wpPosArray.end());

                auto wpVelArray = table->GetEntry("Waypoints/Velocity").GetDoubleArray({});
                waypoint_velocities.assign(wpVelArray.begin(), wpVelArray.end());

                auto wpAccArray = table->GetEntry("Waypoints/Acceleration").GetDoubleArray({});
                waypoint_accelerations.assign(wpAccArray.begin(), wpAccArray.end());

                robotSize = table->GetEntry("RobotSize").GetDouble(0.1);
                calculationDuration = table->GetEntry("CalculationDuration").GetDouble(0.0);
            }
        };
    };
};