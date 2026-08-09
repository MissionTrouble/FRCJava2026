package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class Gyro implements GyroIO {
    private final ADIS16470_IMU imu;

    public Gyro() {
        this.imu = new ADIS16470_IMU();
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(imu.getAngle());
    }

    @Override
    public void reset() {
        imu.reset();
    }

    @Override
    public void update(double speed) {
        // Real hardware updates itself; nothing to integrate here.
    }
}