package frc.robot.subsystems.shooter.mechanisms;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public double velocityRPM = 0.0;
        public double setpointRPM = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void setVelocity(double rpm) {}

    default void stop() {}
}
