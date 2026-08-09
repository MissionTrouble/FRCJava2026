package frc.robot.subsystems.swerve;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class PowerControllerSim implements PowerControllerIO {
    private final SparkMaxSim swervePowerMotor;
    private final SparkRelativeEncoderSim encoder;
    private double lastTime;

    public PowerControllerSim(SparkMax motor) {
        swervePowerMotor = new SparkMaxSim(motor, Constants.getNeoSimMotor());
        encoder = swervePowerMotor.getRelativeEncoderSim();
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void updateInputs(PowerControllerIOInputs inputs) {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        double speedMetersPerSecond = swervePowerMotor.getAppliedOutput() * DriveConstants.MAX_SPEED_MPS;
        double distanceDelta = speedMetersPerSecond * dt;
        encoder.setPosition(encoder.getPosition() + distanceDelta);

        inputs.appliedOutput = swervePowerMotor.getAppliedOutput();
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMetersPerSec = speedMetersPerSecond;
    }

    @Override
    public void run(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        swervePowerMotor.setAppliedOutput(clampedSpeed);
    }

    @Override
    public void stop() {
        swervePowerMotor.setAppliedOutput(0);
    }
}
