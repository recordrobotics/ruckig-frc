package org.recordrobotics.ruckig.jni;

public class RuckigJNI {
    static {
        System.loadLibrary("ruckig_jni");
    }

    // #region Ruckig3
    public static native long create3(double delta_time);

    public static native void destroy3(long handle);

    public static native int update(long handle, long inputHandle, long outputHandle);

    public static native void reset(long handle);

    public static native double getDeltaTime(long handle);

    public static native void setDeltaTime(long handle, double delta_time);
    // #endregion

    // #region InputParameter3
    public static native long createInput3();

    public static native void destroyInput3(long handle);

    public static native int getDurationDiscretization(long handle);

    public static native void setDurationDiscretization(long handle, int discretizationCode);

    public static native int getDefaultSynchronization(long handle);

    public static native void setDefaultSynchronization(long handle, int synchronizationCode);

    public static native int[] getPerDoFSynchronization(long handle);

    public static native void setPerDoFSynchronization(long handle, int[] codes);

    public static native double[] getMaxVelocity(long handle);

    public static native void setMaxVelocity(long handle, double[] maxVelocity);

    public static native double[] getMaxAcceleration(long handle);

    public static native void setMaxAcceleration(long handle, double[] maxAcceleration);

    public static native double[] getMaxJerk(long handle);

    public static native void setMaxJerk(long handle, double[] maxJerk);

    public static native double[] getCurrentPosition(long handle);

    public static native void setCurrentPosition(long handle, double[] currentPosition);

    public static native double[] getCurrentVelocity(long handle);

    public static native void setCurrentVelocity(long handle, double[] currentVelocity);

    public static native double[] getCurrentAcceleration(long handle);

    public static native void setCurrentAcceleration(long handle, double[] currentAcceleration);

    public static native double[] getTargetPosition(long handle);

    public static native void setTargetPosition(long handle, double[] targetPosition);

    public static native double[] getTargetVelocity(long handle);

    public static native void setTargetVelocity(long handle, double[] targetVelocity);

    public static native double[] getTargetAcceleration(long handle);

    public static native void setTargetAcceleration(long handle, double[] targetAcceleration);

    public static native double getMinimumDuration(long handle);

    public static native void setMinimumDuration(long handle, double minimumDuration);

    public static native boolean validate(long handle, boolean check_current_state_within_limits,
            boolean check_target_state_within_limits);

    public static native String toStringInput3(long handle);
    // #endregion

    // #region OutputParameter3
    public static native long createOutput3();

    public static native void destroyOutput3(long handle);

    public static native double[] getNewPosition(long handle);

    public static native double[] getNewVelocity(long handle);

    public static native double[] getNewAcceleration(long handle);

    public static native double[] getNewJerk(long handle);

    public static native double getTime(long handle);

    public static native boolean isNewCalculation(long handle);

    public static native double getCalculationDuration(long handle);

    public static native void passOutputToInput(long outputHandle, long inputHandle);

    public static native String toStringOutput3(long handle);
    // #endregion

    // #region Trajectory3
    public static native double[] trajectoryAtTime(long handle, double time);

    public static native double trajectoryDuration(long handle);

    public static native double[] trajectoryPositionExtrema(long handle);

    public static native double trajectoryFirstTimeAtPosition(long handle, int dof, double position, double time_after);
    // #endregion
}
