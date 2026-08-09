package frc.robot.subsystems.swerve.implementation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.interfaces.PowerControllerIO;
import frc.robot.subsystems.swerve.interfaces.RotationControllerIO;
import frc.robot.subsystems.swerve.simulation.PowerControllerSim;
import frc.robot.subsystems.swerve.simulation.RotationControllerSim;


public class SwerveModule {
    private final PowerControllerIO powerController;
    private final RotationControllerIO rotationController;
    private final String key;
    private double lastTime;

    public SwerveModule(String key, SparkMax power, SparkMax rotation, double offset) {
        this.key = key;
        if (Robot.isReal()) {
            powerController = new PowerController(power);
            rotationController = new RotationController(rotation, offset);
        } else {
            powerController = new PowerControllerSim(power);
            rotationController = new RotationControllerSim(rotation);
        }
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = rotationController.getCompensatedAngle();

        SmartDashboard.putNumber("Swerve/" + key + "/currentAngle", currentAngle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + key + "/goalAngle", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + key + "/preOptimizedVelocity", desiredState.speedMetersPerSecond);

        desiredState.optimize(currentAngle);
        desiredState.cosineScale(currentAngle);

        SmartDashboard.putNumber("Swerve/" + key + "/finalVelocity", desiredState.speedMetersPerSecond);

        double angle = desiredState.angle.getRadians();
        double speed = desiredState.speedMetersPerSecond / DriveConstants.MAX_SPEED_MPS;
        double time = Timer.getFPGATimestamp();
        rotationController.setAngle(angle);
        if (Robot.isReal()) {
            powerController.run(speed);
        } else {
            powerController.run(speed, time - lastTime);
        }

        lastTime = Timer.getFPGATimestamp();
    }

    public SwerveModulePosition getModulePosition() {
        double distance = powerController.getDistance();
        Rotation2d angle = rotationController.getCompensatedAngle();

        return new SwerveModulePosition(distance, angle);
    }

    public SwerveModuleState getModuleState() {
        double velocity = powerController.getSpeed();
        Rotation2d angle = rotationController.getCompensatedAngle();

        return new SwerveModuleState(velocity, angle);
    }

    public double getDistance() {
        return powerController.getDistance();
    }

    public void log() {
        powerController.log(key);
        rotationController.log(key);
    }
}