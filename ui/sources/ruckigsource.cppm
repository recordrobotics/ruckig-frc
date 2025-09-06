module;

#include <array>
#include <ruckig/ruckig.hpp>
#include <imgui.h>

export module ui.sources.ruckigsource;

export namespace ui
{
    namespace sources
    {
        typedef struct Pose2d
        {
            double x, y, r;
            double vx, vy, vr;
            double ax, ay, ar;
            bool editing = false; // Flag to indicate if the waypoint is being edited
            ImColor color;
        } Pose2d;

        class RuckigSource
        {
        public:
            virtual void init() = 0;
            virtual void deinit() = 0;
            virtual void fullReset() = 0;
            virtual void reset() = 0;
            virtual void update() = 0;
            virtual void drawGUI() = 0;
            virtual void drawMenuBar() = 0;

            virtual std::array<double, 3> getActualPosition() const = 0;
            virtual std::array<double, 3> getActualVelocity() const = 0;
            virtual std::array<double, 3> getActualAcceleration() const = 0;
            virtual std::array<double, 3> getSetpointPosition() const = 0;
            virtual std::array<double, 3> getSetpointVelocity() const = 0;
            virtual std::array<double, 3> getSetpointAcceleration() const = 0;
            virtual std::array<double, 3> getTargetPosition() const = 0;
            virtual std::array<double, 3> getTargetVelocity() const = 0;
            virtual std::array<double, 3> getTargetAcceleration() const = 0;
            virtual std::array<double, 3> getNewPosition() const = 0;
            virtual std::array<double, 3> getNewVelocity() const = 0;
            virtual std::array<double, 3> getNewAcceleration() const = 0;
            virtual std::array<double, 3> getNewJerk() const = 0;
            virtual std::array<double, 3> getMaxVelocity() const = 0;
            virtual std::array<double, 3> getMaxAcceleration() const = 0;
            virtual std::array<double, 3> getMaxJerk() const = 0;

            virtual std::array<ruckig::Synchronization, 3> getPerDoFSynchronization() const = 0;
            virtual ruckig::Synchronization getSynchronization() const = 0;
            virtual ruckig::DurationDiscretization getDurationDiscretization() const = 0;

            virtual void updateWaypoints(float x, float y) = 0;
            virtual void editWaypoint(int index, Pose2d wp) = 0;
            virtual std::vector<Pose2d> getWaypoints() = 0;
            virtual void addWaypoint() = 0;
            virtual void removeWaypoint(int index) = 0;

            virtual double getDeltaTime() const = 0;
            virtual double getActualDeltaTime() const = 0;

            virtual double getCalculationTime() const = 0;

            virtual ruckig::Result getResult() const = 0;

            virtual bool canEditWaypoints() const = 0;
            virtual bool isMonotonic() const = 0;

            virtual std::string getTitle() const = 0;
            bool getAndClearTitleDirty()
            {
                if(title_dirty) {
                    title_dirty = false;
                    return true;
                }

                return false;
            }

            void setTitleDirty() { title_dirty = true; }

            virtual double getRobotSize() const = 0;

            int speed = 1;

            std::function<void()> updateCallback = nullptr;
            std::function<void()> resetRobotTrigger = nullptr;

            virtual ~RuckigSource() = default;

        private:
            bool title_dirty = false;
        };
    };
}