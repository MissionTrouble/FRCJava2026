package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}

    /** Sim-only: sets the commanded angular rate used to integrate a fake heading. */
    default void setOmega(double radPerSec) {}

    default void reset() {}
}
