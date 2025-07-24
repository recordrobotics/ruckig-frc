package org.recordrobotics.ruckig;

import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.jni.RuckigJNI;

/**
 * Ruckig instance with 3 degrees of freedom (DoF)
 */
public class Ruckig3 implements AutoCloseable {

    long handle;

    /**
     * Creates a Ruckig instance with 3 degrees of freedom (DoF) and a default delta
     * time of 20 ms.
     */
    public Ruckig3() {
        this(0.02); // Default delta time of 20 ms
    }

    /**
     * Creates a Ruckig instance with 3 degrees of freedom (DoF) and the specified
     * delta time.
     * 
     * @param delta_time The time step for the control loop in seconds.
     */
    public Ruckig3(double delta_time) {
        handle = RuckigJNI.create3(delta_time);
    }

    /**
     * Closes the Ruckig instance and releases the native resources.
     */
    public void close() {
        RuckigJNI.destroy3(handle);
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
     * @param delta_time The new delta time in seconds.
     */
    public void setDeltaTime(double delta_time) {
        RuckigJNI.setDeltaTime(handle, delta_time);
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
        return Result.fromCode(RuckigJNI.update(handle, input.handle, output.handle));
    }
}
