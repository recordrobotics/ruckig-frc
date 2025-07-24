package org.recordrobotics.ruckig;

import org.recordrobotics.ruckig.jni.RuckigJNI;

/**
 * Output of the Ruckig algorithm
 */
public class OutputParameter3 implements AutoCloseable {

    long handle;

    private Trajectory3 trajectory;

    /**
     * Creates an OutputParameter3 instance
     */
    public OutputParameter3() {
        handle = RuckigJNI.createOutput3();
        trajectory = new Trajectory3(handle);
    }

    /**
     * Closes the OutputParameter3 instance and releases the native resources.
     */
    public void close() {
        RuckigJNI.destroyOutput3(handle);
    }

    /**
     * Gets the new position calculated by Ruckig.
     * 
     * @return The new position as an array of doubles for each degree of freedom
     *         (DoF).
     */
    public double[] getNewPosition() {
        return RuckigJNI.getNewPosition(handle);
    }

    /**
     * Gets the new velocity calculated by Ruckig.
     * 
     * @return The new velocity as an array of double for each degree of freedom
     *         (DoF).
     */
    public double[] getNewVelocity() {
        return RuckigJNI.getNewVelocity(handle);
    }

    /**
     * Gets the new acceleration calculated by Ruckig.
     * 
     * @return The new acceleration as an array of double for each degree of
     *         freedom (DoF).
     */
    public double[] getNewAcceleration() {
        return RuckigJNI.getNewAcceleration(handle);
    }

    /**
     * Gets the new jerk calculated by Ruckig.
     * 
     * @return The new jerk as an array of double for each degree of freedom (DoF).
     */
    public double[] getNewJerk() {
        return RuckigJNI.getNewJerk(handle);
    }

    /**
     * Current time on trajectory
     */
    public double getTime() {
        return RuckigJNI.getTime(handle);
    }

    /**
     * Was a new trajectory calculation performed in the last cycle?
     */
    public boolean isNewCalculation() {
        return RuckigJNI.isNewCalculation(handle);
    }

    /**
     * Gets the duration of the last calculation in microseconds (us).
     * 
     * @return The duration of the last calculation in microseconds (us).
     */
    public double getCalculationDuration() {
        return RuckigJNI.getCalculationDuration(handle);
    }

    /**
     * Passes the output parameters to the input parameters for the next update.
     * This is useful if you want to continue the trajectory from the last state.
     * 
     * @param input The input parameters to which the output will be passed.
     */
    public void passToInput(InputParameter3 input) {
        RuckigJNI.passOutputToInput(handle, input.handle);
    }

    /**
     * Gets the trajectory object associated with this output.
     * 
     * @return The trajectory object containing the calculated trajectory.
     */
    public Trajectory3 getTrajectory() {
        return trajectory;
    }

    @Override
    public String toString() {
        return RuckigJNI.toStringOutput3(handle);
    }

}
