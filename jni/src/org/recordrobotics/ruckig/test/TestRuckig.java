package org.recordrobotics.ruckig.test;

import org.recordrobotics.ruckig.InputParameter3;
import org.recordrobotics.ruckig.OutputParameter3;
import org.recordrobotics.ruckig.Ruckig3;
import org.recordrobotics.ruckig.enums.DurationDiscretization;
import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.enums.Synchronization;

public class TestRuckig {

    private static final double[] currentPos = { 0.0, 0.0, 0.0 };
    private static final double[] currentVel = { 0.0, 0.0, 0.0 };
    private static final double[] currentAcc = { 0.0, 0.0, 0.0 };
    private static final double[] targetPos = { 1.0, 1.0, 1.0 };
    private static final double[] targetVel = { 0.0, 0.0, 0.0 };
    private static final double[] targetAcc = { 0.0, 0.0, 0.0 };
    private static final double[] maxVel = { 1.0, 1.0, 1.0 };
    private static final double[] maxAcc = { 1.0, 1.0, 1.0 };
    private static final double[] maxJerk = { 2.2422, 2.1, 2.6 };

    public static void main(String[] args) {
        try (
                Ruckig3 ruckig = new Ruckig3();
                InputParameter3 input = new InputParameter3();
                OutputParameter3 output = new OutputParameter3();) {

            input.setDurationDiscretization(DurationDiscretization.Discrete);
            input.setDefaultSynchronization(Synchronization.Phase);
            input.setPerDoFSynchronization(new Synchronization[] {
                    Synchronization.Phase, Synchronization.Phase, Synchronization.TimeIfNecessary
            });

            input.setMaxVelocity(maxVel);
            input.setMaxAcceleration(maxAcc);
            input.setMaxJerk(maxJerk);

            input.setCurrentPosition(currentPos);
            input.setCurrentVelocity(currentVel);
            input.setCurrentAcceleration(currentAcc);

            input.setTargetPosition(targetPos);
            input.setTargetVelocity(targetVel);
            input.setTargetAcceleration(targetAcc);

            Result result;
            do {
                result = ruckig.update(input, output);
                output.passToInput(input);
            } while (result == Result.Working);

            System.out.println("Result: " + result);
            System.out.println("Input: " + input);
            System.out.println("Output: " + output);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
