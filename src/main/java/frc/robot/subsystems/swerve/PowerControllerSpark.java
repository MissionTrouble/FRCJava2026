package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class PowerControllerSpark implements PowerControllerIO {
    private final SparkMax swervePowerMotor;
    private final RelativeEncoder encoder;

    public PowerControllerSpark(SparkMax motor) {
        swervePowerMotor = motor;
        encoder = swervePowerMotor.getEncoder();
    }

    @Override
    public void updateInputs(PowerControllerIOInputs inputs) {
        inputs.appliedOutput = swervePowerMotor.get();
        inputs.positionMeters = encoder.getPosition();
        // Preserves original PowerController.getSpeed() behavior: raw commanded output, not a
        // true encoder-derived velocity (the drive encoder has no meters-per-rotation conversion
        // factor configured).
        inputs.velocityMetersPerSec = swervePowerMotor.get();
    }

    @Override
    public void run(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        swervePowerMotor.set(clampedSpeed);
    }

    @Override
    public void stop() {
        swervePowerMotor.set(0);
    }
}
