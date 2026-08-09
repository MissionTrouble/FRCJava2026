package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final PowerControllerIO powerController;
    private final RotationControllerIO rotationController;
    private final PowerControllerIOInputsAutoLogged powerInputs = new PowerControllerIOInputsAutoLogged();
    private final RotationControllerIOInputsAutoLogged rotationInputs = new RotationControllerIOInputsAutoLogged();
    private final String key;

    public SwerveModule(String key, PowerControllerIO powerController, RotationControllerIO rotationController) {
        this.key = key;
        this.powerController = powerController;
        this.rotationController = rotationController;
    }

    public void updateInputs() {
        powerController.updateInputs(powerInputs);
        rotationController.updateInputs(rotationInputs);
        Logger.processInputs("Swerve/" + key + "/Power", powerInputs);
        Logger.processInputs("Swerve/" + key + "/Rotation", rotationInputs);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = new Rotation2d(rotationInputs.compensatedAngleRad);

        Logger.recordOutput("Swerve/" + key + "/currentAngle", currentAngle.getDegrees());
        Logger.recordOutput("Swerve/" + key + "/goalAngle", desiredState.angle.getDegrees());
        Logger.recordOutput("Swerve/" + key + "/preOptimizedVelocity", desiredState.speedMetersPerSecond);

        desiredState.optimize(currentAngle);
        desiredState.cosineScale(currentAngle);

        Logger.recordOutput("Swerve/" + key + "/finalVelocity", desiredState.speedMetersPerSecond);

        double angle = desiredState.angle.getRadians();
        double speed = desiredState.speedMetersPerSecond / DriveConstants.MAX_SPEED_MPS;
        rotationController.setAngle(angle);
        powerController.run(speed);
    }

    public SwerveModulePosition getModulePosition() {
        double distance = powerInputs.positionMeters;
        Rotation2d angle = new Rotation2d(rotationInputs.compensatedAngleRad);

        return new SwerveModulePosition(distance, angle);
    }

    public SwerveModuleState getModuleState() {
        double velocity = powerInputs.velocityMetersPerSec;
        Rotation2d angle = new Rotation2d(rotationInputs.compensatedAngleRad);

        return new SwerveModuleState(velocity, angle);
    }

    public double getDistance() {
        return powerInputs.positionMeters;
    }
}
