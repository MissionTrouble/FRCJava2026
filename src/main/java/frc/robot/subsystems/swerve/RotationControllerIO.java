package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface RotationControllerIO {
    @AutoLog
    class RotationControllerIOInputs {
        public double rawAngleRad = 0.0;
        public double compensatedAngleRad = 0.0;
        public double setpointRad = 0.0;
    }

    default void updateInputs(RotationControllerIOInputs inputs) {}

    default void setAngle(double setpointRad) {}

    default void stop() {}
}
