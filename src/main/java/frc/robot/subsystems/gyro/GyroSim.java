package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroSim implements GyroIO {
    private double angleRadians;
    private double lastTime;

    public GyroSim() {
        this(0.0);
    }

    public GyroSim(double startingAngleRadians) {
        this.angleRadians = startingAngleRadians;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(angleRadians);
    }

    @Override
    public void reset() {
        angleRadians = 0;
    }

    @Override
    public void update(double speed) {
        double time = Timer.getFPGATimestamp();
        double elapsed = time - lastTime;

        angleRadians -= speed * elapsed;
        SmartDashboard.putNumber("Gyro/simAngleRadians", angleRadians);
        lastTime = time;
    }
}