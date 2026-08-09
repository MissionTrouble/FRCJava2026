package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroIOReal implements GyroIO {
    private final ADIS16470_IMU imu = new ADIS16470_IMU();

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = imu.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(imu.getAngle());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(imu.getRate());
    }

    @Override
    public void reset() {
        imu.reset();
    }
}
