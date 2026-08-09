package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface PowerControllerIO {
    @AutoLog
    class PowerControllerIOInputs {
        public double appliedOutput = 0.0;
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
    }

    default void updateInputs(PowerControllerIOInputs inputs) {}

    default void run(double speed) {}

    default void stop() {}
}
