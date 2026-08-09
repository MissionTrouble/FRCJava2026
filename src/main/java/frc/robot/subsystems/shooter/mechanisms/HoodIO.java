package frc.robot.subsystems.shooter.mechanisms;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double positionRad = 0.0;
        public double setpointRad = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void setPosition(double setpoint) {}

    default void stop() {}
}
