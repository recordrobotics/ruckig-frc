package org.recordrobotics.ruckig.wpi;

import org.recordrobotics.ruckig.enums.DurationDiscretization;
import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.enums.Synchronization;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Telemetry for Ruckig using NetworkTables.
 */
public final class RuckigTelemetry implements AutoCloseable {
    
    private final String name;
    private final NetworkTable table;

    /**
     * Creates a RuckigTelemetry instance with the specified name and degrees of freedom (DoF).
     * @param name the name of the Ruckig instance
     * @param dof the degrees of freedom (only for initializing default values)
     */
    public RuckigTelemetry(String name, int dof) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable("Ruckig/" + name);
        putDefaultValues(dof);
    }

    /**
     * Gets the name of the Ruckig instance.
     * @return the name of the Ruckig instance
     */
    public String getName() {
        return name;
    }

    @Override
    public void close() throws Exception {
        putActive(false);
    }

    /*
     * Puts default values for the specified degrees of freedom (DoF).
     */
    public void putDefaultValues(int dof) {
        putActualPosition(new double[dof]);
        putActualVelocity(new double[dof]);
        putSetpointPosition(new double[dof]);
        putSetpointVelocity(new double[dof]);
        putSetpointAcceleration(new double[dof]);
        putTargetPosition(new double[dof]);
        putTargetVelocity(new double[dof]);
        putTargetAcceleration(new double[dof]);
        putNewPosition(new double[dof]);
        putNewVelocity(new double[dof]);
        putNewAcceleration(new double[dof]);
        putNewJerk(new double[dof]);

        putMaxVelocity(new double[dof]);
        
        double[] maxAcceleration = new double[dof];
        java.util.Arrays.fill(maxAcceleration, Double.POSITIVE_INFINITY);
        putMaxAcceleration(maxAcceleration);
        
        double[] maxJerk = new double[dof];
        java.util.Arrays.fill(maxJerk, Double.POSITIVE_INFINITY);
        putMaxJerk(maxJerk);

        Synchronization[] sync = new Synchronization[dof];
        java.util.Arrays.fill(sync, Synchronization.Time);
        putPerDoFSynchronization(sync);
        putDefaultSynchronization(Synchronization.Time);
        putDurationDiscretization(DurationDiscretization.Continuous);
        putCalculationDuration(0);
        putResult(Result.Working);
        putActive(true);
    }

    public void putActive(boolean active) {
        table.getEntry("Active").setBoolean(active);
    }

    public void putActualPosition(double[] actualPosition) {
        table.getEntry("ActualPosition").setDoubleArray(actualPosition);
    }

    public void putActualVelocity(double[] actualVelocity) {
        table.getEntry("ActualVelocity").setDoubleArray(actualVelocity);
    }

    public void putSetpointPosition(double[] setpointPosition) {
        table.getEntry("SetpointPosition").setDoubleArray(setpointPosition);
    }

    public void putSetpointVelocity(double[] setpointVelocity) {
        table.getEntry("SetpointVelocity").setDoubleArray(setpointVelocity);
    }

    public void putSetpointAcceleration(double[] setpointAcceleration) {
        table.getEntry("SetpointAcceleration").setDoubleArray(setpointAcceleration);
    }

    public void putTargetPosition(double[] targetPosition) {
        table.getEntry("TargetPosition").setDoubleArray(targetPosition);
    }

    public void putTargetVelocity(double[] targetVelocity) {
        table.getEntry("TargetVelocity").setDoubleArray(targetVelocity);
    }

    public void putTargetAcceleration(double[] targetAcceleration) {
        table.getEntry("TargetAcceleration").setDoubleArray(targetAcceleration);
    }

    public void putNewPosition(double[] newPosition) {
        table.getEntry("NewPosition").setDoubleArray(newPosition);
    }

    public void putNewVelocity(double[] newVelocity) {
        table.getEntry("NewVelocity").setDoubleArray(newVelocity);
    }

    public void putNewAcceleration(double[] newAcceleration) {
        table.getEntry("NewAcceleration").setDoubleArray(newAcceleration);
    }

    public void putNewJerk(double[] newJerk) {
        table.getEntry("NewJerk").setDoubleArray(newJerk);
    }

    public void putMaxVelocity(double[] maxVelocity) {
        table.getEntry("MaxVelocity").setDoubleArray(maxVelocity);
    }

    public void putMaxAcceleration(double[] maxAcceleration) {
        table.getEntry("MaxAcceleration").setDoubleArray(maxAcceleration);
    }

    public void putMaxJerk(double[] maxJerk) {
        table.getEntry("MaxJerk").setDoubleArray(maxJerk);
    }

    public void putPerDoFSynchronization(Synchronization[] synchronization) {
        long[] codes = new long[synchronization.length];
        for (int i = 0; i < synchronization.length; i++) {
            codes[i] = synchronization[i].getCode();
        }
        table.getEntry("PerDoFSynchronization").setIntegerArray(codes);
    }

    public void putDefaultSynchronization(Synchronization synchronization) {
        table.getEntry("DefaultSynchronization").setInteger(synchronization.getCode());
    }

    public void putDurationDiscretization(DurationDiscretization durationDiscretization) {
        table.getEntry("DurationDiscretization").setInteger(durationDiscretization.getCode());
    }

    public void putCalculationDuration(double calculationDuration) {
        table.getEntry("CalculationDuration").setDouble(calculationDuration);
    }

    public void putResult(Result result) {
        table.getEntry("Result").setInteger(result.getCode());
    }

    public void putRobotSize(double robotSize) {
        table.getEntry("RobotSize").setDouble(robotSize);
    }

    @SuppressWarnings("java:S6218")
    public static record WaypointData(double[] position, double[] velocity, double[] acceleration) {}

    public void putWaypoints(WaypointData[] waypoints) {
        int dof = waypoints[0].position.length;
        int numWaypoints = waypoints.length;

        double[] positions = new double[numWaypoints * dof];
        double[] velocities = new double[numWaypoints * dof];
        double[] accelerations = new double[numWaypoints * dof];

        for (int i = 0; i < numWaypoints; i++) {
            System.arraycopy(waypoints[i].position, 0, positions, i * dof, dof);
            System.arraycopy(waypoints[i].velocity, 0, velocities, i * dof, dof);
            System.arraycopy(waypoints[i].acceleration, 0, accelerations, i * dof, dof);
        }

        table.getEntry("Waypoints/Position").setDoubleArray(positions);
        table.getEntry("Waypoints/Velocity").setDoubleArray(velocities);
        table.getEntry("Waypoints/Acceleration").setDoubleArray(accelerations);
        table.getEntry("Waypoints/DOF").setInteger(dof);
    }

}
