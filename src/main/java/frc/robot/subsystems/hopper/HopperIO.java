package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    class HopperIOInputs {
        public double appliedOutput = 0.0;
    }

    default void updateInputs(HopperIOInputs inputs) {}

    default void set(double speed) {}

    default void stop() {}
}
