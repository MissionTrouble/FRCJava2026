package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    Rotation2d getRotation2d();
    void update(double speed);
    void reset();
}