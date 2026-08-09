package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
    @AutoLog
    class IntakeRollerIOInputs {
        public double appliedOutput = 0.0;
    }

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    default void set(double speed) {}

    default void stop() {}
}
