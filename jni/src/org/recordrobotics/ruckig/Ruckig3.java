package org.recordrobotics.ruckig;

import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.jni.RuckigJNI;
import org.recordrobotics.ruckig.wpi.RuckigTelemetry;
import org.recordrobotics.ruckig.wpi.RuckigTelemetry.WaypointData;

/**
 * Ruckig instance with 3 degrees of freedom (DoF)
 */
public class Ruckig3 implements AutoCloseable {

    private final long handle;
    private final RuckigTelemetry telemetry;

    /**
     * Creates a Ruckig instance with 3 degrees of freedom (DoF) and a default delta
     * time of 20 ms.
     */
    public Ruckig3() {
        this(null, 0.02); // Default delta time of 20 ms
    }

    /**
     * Creates a Ruckig instance with 3 degrees of freedom (DoF) and the specified
     * delta time.
     * 
     * @param telemetryName The name for telemetry purposes (if null then auto-generated).
     * @param deltaTime The time step for the control loop in seconds.
     */
    public Ruckig3(String telemetryName, double deltaTime) {
        handle = RuckigJNI.create3(deltaTime);
        telemetry = new RuckigTelemetry(telemetryName, 3);
    }

    /**
     * Closes the Ruckig instance and releases the native resources.
     */
    @Override
    public void close() throws Exception {
        RuckigJNI.destroy3(handle);
        telemetry.close();
    }

    /**
     * Reset the instance (e.g. to force a new calculation in the next update)
     */
    public void reset() {
        RuckigJNI.reset(handle);
    }

    /**
     * Gets the delta time of the Ruckig instance.
     * 
     * @return The delta time in seconds.
     */
    public double getDeltaTime() {
        return RuckigJNI.getDeltaTime(handle);
    }

    /**
     * Sets the delta time of the Ruckig instance.
     * 
     * @param deltaTime The new delta time in seconds.
     */
    public void setDeltaTime(double deltaTime) {
        RuckigJNI.setDeltaTime(handle, deltaTime);
    }

    /**
     * Get the next output state (with step delta_time) along the calculated
     * trajectory for the given input
     * 
     * @param input  The input parameters for the Ruckig calculation.
     * @param output The output parameters where the result will be stored.
     * @return The result of the update operation. Check this to determine if the
     *         trajectory is finished.
     */
    public Result update(InputParameter3 input, OutputParameter3 output) {
        Result result = Result.fromCode(RuckigJNI.update(handle, input.handle, output.handle));
        updateTelemetry(result, input, output);
        return result;
    }

    private void updateTelemetry(Result result, InputParameter3 input, OutputParameter3 output) {
        telemetry.putResult(result);
        telemetry.putSetpointPosition(input.getCurrentPosition());
        telemetry.putSetpointVelocity(input.getCurrentVelocity());
        telemetry.putSetpointAcceleration(input.getCurrentAcceleration());
        telemetry.putTargetPosition(input.getTargetPosition());
        telemetry.putTargetVelocity(input.getTargetVelocity());
        telemetry.putTargetAcceleration(input.getTargetAcceleration());
        telemetry.putNewPosition(output.getNewPosition());
        telemetry.putNewVelocity(output.getNewVelocity());
        telemetry.putNewAcceleration(output.getNewAcceleration());
        telemetry.putNewJerk(output.getNewJerk());
        telemetry.putMaxVelocity(input.getMaxVelocity());
        telemetry.putMaxAcceleration(input.getMaxAcceleration());
        telemetry.putMaxJerk(input.getMaxJerk());
        telemetry.putPerDoFSynchronization(input.getPerDoFSynchronization());
        telemetry.putDefaultSynchronization(input.getDefaultSynchronization());
        telemetry.putDurationDiscretization(input.getDurationDiscretization());
        telemetry.putCalculationDuration(output.getCalculationDuration());
    }

    /**
     * Update the telemetry from the real system (e.g. from odometry)
     * @param position The actual position of the system
     * @param velocity The actual velocity of the system
     * @param robotSize The size of the robot
     */
    public void updateTelemetry(double[] position, double[] velocity, double robotSize) {
        telemetry.putActualPosition(position);
        telemetry.putActualVelocity(velocity);
        telemetry.putRobotSize(robotSize);
    }

    public void publishWaypointsToTelemetry(WaypointData[] waypoints) {
        telemetry.putWaypoints(waypoints);
    }
}
