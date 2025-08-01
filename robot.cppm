module;

#include <spdlog/spdlog.h>

export module robot;

export class Robot
{
public:
    Robot() : pos_x(0.0), pos_y(0.0), rot(0.0),
              last_pos_x(0.0), last_pos_y(0.0), last_rot(0.0),
              last_vel_x(0.0), last_vel_y(0.0), last_vel_rot(0.0)
    {
        spdlog::info("Robot initialized at position ({}, {}) with rotation {}", pos_x, pos_y, rot);
    }

    void setPosition(double x, double y, double r)
    {
        pos_x = x;
        pos_y = y;
        rot = r;
    }

    void getPosition(double &x, double &y, double &r) const
    {
        x = pos_x;
        y = pos_y;
        r = rot;
    }

    void update(double delta_time, double &x, double &y, double &r, double &vx, double &vy, double &vrot, double &ax, double &ay, double &arot)
    {
        pos_x += vx * delta_time;
        pos_y += vy * delta_time;
        rot += vrot * delta_time;

        x = pos_x;
        y = pos_y;
        r = rot;
        vx = (pos_x - last_pos_x) / delta_time;
        vy = (pos_y - last_pos_y) / delta_time;
        vrot = (rot - last_rot) / delta_time;
        ax = (vx - last_vel_x) / delta_time;
        ay = (vy - last_vel_y) / delta_time;
        arot = (vrot - last_vel_rot) / delta_time;
        last_pos_x = pos_x;
        last_pos_y = pos_y;
        last_rot = rot;
        last_vel_x = vx;
        last_vel_y = vy;
        last_vel_rot = vrot;
    }

private:
    double pos_x;
    double pos_y;
    double rot;

    double last_pos_x;
    double last_pos_y;
    double last_rot;
    double last_vel_x;
    double last_vel_y;
    double last_vel_rot;
};