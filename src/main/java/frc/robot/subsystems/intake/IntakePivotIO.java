package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
    @AutoLog
    class IntakePivotIOInputs {
        public double appliedOutput = 0.0;
        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;
    }

    default void updateInputs(IntakePivotIOInputs inputs) {}

    /** Signed speed: positive pivots up, negative pivots down. */
    default void set(double speed) {}

    default void stop() {}
}
