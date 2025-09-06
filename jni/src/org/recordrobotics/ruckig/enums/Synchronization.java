package org.recordrobotics.ruckig.enums;

public enum Synchronization {
    /// Always synchronize the DoFs to reach the target at the same time (Default)
    Time(0),
    /// Synchronize only when necessary (e.g. for non-zero target velocity or acceleration)
    TimeIfNecessary(1),
    /// Phase synchronize the DoFs when this is possible, else fallback to "Time" strategy. Phase synchronization will result a straight-line trajectory
    Phase(2),
    /// Calculate every DoF independently
    None(3);

    private final int code;

    Synchronization(int code) {
        this.code = code;
    }

    public int getCode() {
        return code;
    }

    public static Synchronization fromCode(int code) {
        for (Synchronization val : Synchronization.values()) {
            if (val.getCode() == code) {
                return val;
            }
        }
        return Time; // Default to Time if no match found
    }
}
