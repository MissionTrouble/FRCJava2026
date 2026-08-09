package frc.robot.subsystems.shooter.mechanisms;

import org.littletonrobotics.junction.AutoLog;

public interface ChuteIO {
    @AutoLog
    class ChuteIOInputs {
        public double appliedOutput = 0.0;
    }

    default void updateInputs(ChuteIOInputs inputs) {}

    default void set(double speed) {}

    default void stop() {}
}
