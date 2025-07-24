package org.recordrobotics.ruckig.enums;

/**
 * Result type of Ruckig's update function
 */
public enum Result {
    /// The trajectory is calculated normally
    Working(0),
    /// The trajectory has reached its final position
    Finished(1),
    /// Unclassified error
    Error(-1),
    /// Error in the input parameter
    ErrorInvalidInput(-100), 
    /// The trajectory duration exceeds its numerical limits
    ErrorTrajectoryDuration(-101),
    /// The trajectory exceeds the given positional limits (only in Ruckig Pro)
    ErrorPositionalLimits(-102), 
    /// The trajectory cannot be phase synchronized
    ErrorNoPhaseSynchronization(-103), 
    /// The trajectory is not valid due to a conflict with zero limits
    ErrorZeroLimits(-104),
    /// Error during the extremel time calculation (Step 1)
    ErrorExecutionTimeCalculation(-110), 
    /// Error during the synchronization calculation (Step 2)
    ErrorSynchronizationCalculation(-111);
    
    private final int code;
    Result(int code) {
        this.code = code;
    }

    public int getCode() {
        return code;
    }

    public static Result fromCode(int code) {
        for (Result result : Result.values()) {
            if (result.getCode() == code) {
                return result;
            }
        }
        return Error; // Default to Error if no match found
    }
}
