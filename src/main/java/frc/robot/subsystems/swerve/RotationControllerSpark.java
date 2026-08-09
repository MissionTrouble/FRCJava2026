package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

public class RotationControllerSpark implements RotationControllerIO {
    private final SparkMax swerveRotationMotor;
    private final SparkClosedLoopController swerveRotationController;
    private final double offset;

    public RotationControllerSpark(SparkMax motor, double offset) {
        this.offset = offset;
        swerveRotationMotor = motor;
        swerveRotationController = swerveRotationMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(RotationControllerIOInputs inputs) {
        inputs.rawAngleRad = swerveRotationMotor.getAnalog().getPosition();
        inputs.compensatedAngleRad = swerveRotationMotor.getAnalog().getPosition() - offset;
        inputs.setpointRad = swerveRotationController.getSetpoint();
    }

    @Override
    public void setAngle(double setpointRad) {
        swerveRotationController.setSetpoint(setpointRad - offset, ControlType.kPosition);
    }

    @Override
    public void stop() {
        swerveRotationMotor.set(0);
    }
}
