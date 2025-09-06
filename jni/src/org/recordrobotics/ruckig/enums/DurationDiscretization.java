package org.recordrobotics.ruckig.enums;

public enum DurationDiscretization {
    /// Every trajectory synchronization duration is allowed (Default)
    Continuous(0),
    /// The trajectory synchronization duration must be a multiple of the control cycle
    Discrete(1);

    private final int code;

    DurationDiscretization(int code) {
        this.code = code;
    }

    public int getCode() {
        return code;
    }

    public static DurationDiscretization fromCode(int code) {
        for (DurationDiscretization val : DurationDiscretization.values()) {
            if (val.getCode() == code) {
                return val;
            }
        }
        return Continuous; // Default to Continuous if no match found
    }
}
