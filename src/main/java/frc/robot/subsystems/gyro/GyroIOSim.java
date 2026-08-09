package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
    private double angleRadians;
    private double omegaRadPerSec = 0.0;
    private double lastTime;

    public GyroIOSim() {
        this(0.0);
    }

    public GyroIOSim(double startingAngleRadians) {
        this.angleRadians = startingAngleRadians;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double time = Timer.getFPGATimestamp();
        double elapsed = time - lastTime;
        lastTime = time;

        angleRadians -= omegaRadPerSec * elapsed;

        inputs.connected = true;
        inputs.yawPosition = new Rotation2d(angleRadians);
        inputs.yawVelocityRadPerSec = -omegaRadPerSec;
    }

    @Override
    public void setOmega(double radPerSec) {
        this.omegaRadPerSec = radPerSec;
    }

    @Override
    public void reset() {
        angleRadians = 0;
    }
}
