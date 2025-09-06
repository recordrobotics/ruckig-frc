package org.recordrobotics.ruckig;

import java.util.OptionalDouble;

import org.recordrobotics.ruckig.enums.DurationDiscretization;
import org.recordrobotics.ruckig.enums.Synchronization;
import org.recordrobotics.ruckig.jni.RuckigJNI;

/**
 * Input of the Ruckig algorithm with 3 degrees of freedom (DoF)
 */
public class InputParameter3 implements AutoCloseable {

    long handle;

    /**
     * Creates an InputParameter3 instance with default settings.
     */
    public InputParameter3() {
        handle = RuckigJNI.createInput3();
    }

    /**
     * Closes the InputParameter3 instance and releases the native resources.
     */
    public void close() {
        RuckigJNI.destroyInput3(handle);
    }

    /**
     * Gets the duration discretization of the input.
     */
    public DurationDiscretization getDurationDiscretization() {
        return DurationDiscretization.fromCode(RuckigJNI.getDurationDiscretization(handle));
    }

    /**
     * Sets the duration discretization of the input.
     *
     * @param discretization The duration discretization to set.
     */
    public void setDurationDiscretization(DurationDiscretization discretization) {
        RuckigJNI.setDurationDiscretization(handle, discretization.getCode());
    }

    /**
     * Gets the default synchronization mode for the input.
     */
    public Synchronization getDefaultSynchronization() {
        return Synchronization.fromCode(RuckigJNI.getDefaultSynchronization(handle));
    }

    /**
     * Sets the default synchronization mode for the input.
     *
     * @param synchronization The synchronization mode to set.
     */
    public void setDefaultSynchronization(Synchronization synchronization) {
        RuckigJNI.setDefaultSynchronization(handle, synchronization.getCode());
    }

    /**
     * Gets the synchronization mode per degree of freedom (DoF) for the input.
     */
    public Synchronization[] getPerDoFSynchronization() {
        int[] codes = RuckigJNI.getPerDoFSynchronization(handle);
        Synchronization[] synchronizations = new Synchronization[codes.length];
        for (int i = 0; i < codes.length; i++) {
            synchronizations[i] = Synchronization.fromCode(codes[i]);
        }
        return synchronizations;
    }

    /**
     * Sets the synchronization mode per degree of freedom (DoF) for the input.
     *
     * @param synchronizations The array of Synchronization modes to set.
     */
    public void setPerDoFSynchronization(Synchronization[] synchronizations) {
        int[] codes = new int[synchronizations.length];
        for (int i = 0; i < synchronizations.length; i++) {
            codes[i] = synchronizations[i].getCode();
        }
        RuckigJNI.setPerDoFSynchronization(handle, codes);
    }

    /**
     * Gets the maximum velocity for the input.
     */
    public double[] getMaxVelocity() {
        return RuckigJNI.getMaxVelocity(handle);
    }

    /**
     * Sets the maximum velocity for the input.
     *
     * @param values The array of maximum velocities to set.
     */
    public void setMaxVelocity(double[] values) {
        RuckigJNI.setMaxVelocity(handle, values);
    }

    /**
     * Gets the maximum acceleration for the input.
     */
    public double[] getMaxAcceleration() {
        return RuckigJNI.getMaxAcceleration(handle);
    }

    /**
     * Sets the maximum acceleration for the input.
     *
     * @param values The array of maximum accelerations to set.
     */
    public void setMaxAcceleration(double[] values) {
        RuckigJNI.setMaxAcceleration(handle, values);
    }

    /**
     * Gets the maximum jerk for the input.
     */
    public double[] getMaxJerk() {
        return RuckigJNI.getMaxJerk(handle);
    }

    /**
     * Sets the maximum jerk for the input.
     *
     * @param values The array of maximum jerks to set.
     */
    public void setMaxJerk(double[] values) {
        RuckigJNI.setMaxJerk(handle, values);
    }

    /**
     * Gets the current position for the input.
     */
    public double[] getCurrentPosition() {
        return RuckigJNI.getCurrentPosition(handle);
    }

    /**
     * Sets the current position for the input.
     *
     * @param values The array of current positions to set.
     */
    public void setCurrentPosition(double[] values) {
        RuckigJNI.setCurrentPosition(handle, values);
    }

    /**
     * Gets the current velocity for the input.
     */
    public double[] getCurrentVelocity() {
        return RuckigJNI.getCurrentVelocity(handle);
    }

    /**
     * Sets the current velocity for the input.
     *
     * @param values The array of current velocities to set.
     */
    public void setCurrentVelocity(double[] values) {
        RuckigJNI.setCurrentVelocity(handle, values);
    }

    /**
     * Gets the current acceleration for the input.
     */
    public double[] getCurrentAcceleration() {
        return RuckigJNI.getCurrentAcceleration(handle);
    }

    /**
     * Sets the current acceleration for the input.
     *
     * @param values The array of current accelerations to set.
     */
    public void setCurrentAcceleration(double[] values) {
        RuckigJNI.setCurrentAcceleration(handle, values);
    }

    /**
     * Gets the target position for the input.
     */
    public double[] getTargetPosition() {
        return RuckigJNI.getTargetPosition(handle);
    }

    /**
     * Sets the target position for the input.
     *
     * @param values The array of target positions to set.
     */
    public void setTargetPosition(double[] values) {
        RuckigJNI.setTargetPosition(handle, values);
    }

    /**
     * Gets the target velocity for the input.
     */
    public double[] getTargetVelocity() {
        return RuckigJNI.getTargetVelocity(handle);
    }

    /**
     * Sets the target velocity for the input.
     *
     * @param values The array of target velocities to set.
     */
    public void setTargetVelocity(double[] values) {
        RuckigJNI.setTargetVelocity(handle, values);
    }

    /**
     * Gets the target acceleration for the input.
     */
    public double[] getTargetAcceleration() {
        return RuckigJNI.getTargetAcceleration(handle);
    }

    /**
     * Sets the target acceleration for the input.
     *
     * @param values The array of target accelerations to set.
     */
    public void setTargetAcceleration(double[] values) {
        RuckigJNI.setTargetAcceleration(handle, values);
    }

    /**
     * Gets the optional minimum duration for the input.
     *
     * @return An OptionalDouble containing the minimum duration, or empty if not
     *         set.
     */
    public OptionalDouble getMinimumDuration() {
        double duration = RuckigJNI.getMinimumDuration(handle);
        return duration != Double.MIN_VALUE ? OptionalDouble.of(duration) : OptionalDouble.empty();
    }

    /**
     * Sets the optional minimum duration for the input.
     *
     * @param duration An OptionalDouble containing the minimum duration to set, or
     *                 empty to unset.
     */
    public void setMinimumDuration(OptionalDouble duration) {
        if (duration.isPresent()) {
            RuckigJNI.setMinimumDuration(handle, duration.getAsDouble());
        } else {
            RuckigJNI.setMinimumDuration(handle, Double.MIN_VALUE);
        }
    }

    /**
     * Validate the input for trajectory calculation
     * 
     * @return true if the input is valid, false otherwise.
     */
    public boolean validate() {
        return validate(false, true);
    }

    /**
     * Validate the input for trajectory calculation
     * 
     * @param check_current_state_within_limits if true, checks if the current state
     *                                          is within limits
     * @param check_target_state_within_limits  if true, checks if the target state
     *                                          is within limits
     * @return true if the input is valid, false otherwise.
     */
    public boolean validate(boolean check_current_state_within_limits, boolean check_target_state_within_limits) {
        return RuckigJNI.validate(handle, check_current_state_within_limits, check_target_state_within_limits);
    }

    @Override
    public String toString() {
        return RuckigJNI.toStringInput3(handle);
    }
}
