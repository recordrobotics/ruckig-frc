module;

#include <spdlog/spdlog.h>

export module pid;

export class PID
{
public:
    PID()
    {
    }

    void reset()
    {
        integral = 0.0;
        last_error = 0.0;
    }

    double calculate(double setpoint, double measured_value, double kp, double ki, double kd, double delta_time)
    {
        double error = setpoint - measured_value;
        integral += error * delta_time;
        double derivative = (error - last_error) / delta_time;
        last_error = error;

        return kp * error + ki * integral + kd * derivative;
    }

private:
    double integral = 0.0;   // Integral term for PID
    double last_error = 0.0; // Last error for derivative calculation
};