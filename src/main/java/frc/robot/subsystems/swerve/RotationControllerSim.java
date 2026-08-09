package frc.robot.subsystems.swerve;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class RotationControllerSim implements RotationControllerIO {
    private final SparkMaxSim swerveRotationMotor;
    private final SparkAbsoluteEncoderSim swerveRotationEncoder;

    public RotationControllerSim(SparkMax motor) {
        swerveRotationMotor = new SparkMaxSim(motor, Constants.getNeoSimMotor());
        swerveRotationEncoder = swerveRotationMotor.getAbsoluteEncoderSim();
    }

    @Override
    public void updateInputs(RotationControllerIOInputs inputs) {
        inputs.rawAngleRad = swerveRotationEncoder.getPosition();
        inputs.compensatedAngleRad = swerveRotationEncoder.getPosition();
        inputs.setpointRad = swerveRotationEncoder.getPosition();
    }

    @Override
    public void setAngle(double setpointRad) {
        swerveRotationEncoder.setPosition(setpointRad);
    }

    @Override
    public void stop() {}
}
