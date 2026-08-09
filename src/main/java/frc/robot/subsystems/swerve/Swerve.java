package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.odometry.Odometry;

public class Swerve {
    private final DoubleSupplier xSupplier, ySupplier, oSupplier, triggerSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public final SwerveSubsystem swerve;
    private final Odometry odometry;

    private final StructTopic<ChassisSpeeds> chassisSpeedsTopic = NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/ChassisVelocityJoystick", ChassisSpeeds.struct);
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = chassisSpeedsTopic.publish();

    private final StructArrayTopic<SwerveModuleState> swerveStateJoystickTopic =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SmartDashboard/Joystick/SwerveVelocity", SwerveModuleState.struct);
    private final StructArrayPublisher<SwerveModuleState> swerveStateJoystickPublisher = swerveStateJoystickTopic.publish();

    private final PIDController hubRotationPID = new PIDController(1.8, 0, 0.5); // TODO: tune
    InterpolatingDoubleTreeMap shootTimeCalibration = new InterpolatingDoubleTreeMap();


    public Swerve(Odometry odometry, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier oSupplier, DoubleSupplier triggerSupplier) {
        this.odometry = odometry;
        swerve = new SwerveSubsystem(odometry);
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.oSupplier = oSupplier;
        this.triggerSupplier = triggerSupplier;
        this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
        this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.MAX_ANGULAR_ACCELERATION);
        hubRotationPID.enableContinuousInput(-Math.PI, Math.PI);
        shootTimeCalibration.put(1.0, 0.25);
        shootTimeCalibration.put(2.0, 0.4);
        shootTimeCalibration.put(3.0, 0.6);
    }

    public Command joystickDriveCommand() {
        return Commands.run(this::joystickDrive, swerve).withName("JoystickDrive");
    }

    public void joystickDrive() {
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double omegaSpeed = oSupplier.getAsDouble();
        double boost = triggerSupplier.getAsDouble();

        if (DriveConstants.TRIGGER_BOOST) {
            xSpeed /= 2;
            xSpeed += xSpeed * boost;
            ySpeed /= 2;
            ySpeed += ySpeed * boost;
            omegaSpeed /= 2;
            omegaSpeed += omegaSpeed * boost;
        }

        xSpeed = xLimiter.calculate(xSpeed * DriveConstants.MAX_SPEED_MPS);
        ySpeed = yLimiter.calculate(ySpeed * DriveConstants.MAX_SPEED_MPS);
        omegaSpeed = turningLimiter.calculate(omegaSpeed * DriveConstants.MAX_ANGULAR_SPEED_MPS);
        SmartDashboard.putNumber("Joystick/xSpeed", xSpeed);
        SmartDashboard.putNumber("Joystick/ySpeed", ySpeed);
        SmartDashboard.putNumber("Joystick/omegaSpeed", omegaSpeed);

        if (!Robot.isReal()) {
            odometry.updateGyro(omegaSpeed);
        }

        ChassisSpeeds chassisSpeeds;
        if (DriveConstants.FIELD_ORIENTED) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, odometry.getHeading());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
        }

        chassisSpeedsPublisher.set(chassisSpeeds);
        SwerveModuleState[] states = DriveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveStateJoystickPublisher.set(states);
        swerve.setDesiredStates(states);
    }

    public Command pointToHubCommand() {
        return Commands.run(this::pointToHub, swerve).withName("PointToHub");
    }

    public void pointToHub() {
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double boost = triggerSupplier.getAsDouble();

        Translation2d robotTranslation = odometry.getPose().getTranslation();
        double distance = Constants.HUB_LOCATION.getDistance(robotTranslation);

        ChassisSpeeds robotRelativeSpeeds = DriveConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(swerve.getModuleStates());
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, odometry.getHeading());
        Translation2d velocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        Translation2d aimVector = velocity.times(shootTimeCalibration.get(distance));
        Translation2d aimPosition = Constants.HUB_LOCATION.minus(robotTranslation).minus(aimVector);

        Rotation2d desiredHeading = aimPosition.getAngle();
        double currentHeadingRad = odometry.getHeading().getRadians();
        double omegaSpeed = -hubRotationPID.calculate(currentHeadingRad, desiredHeading.getRadians());
        omegaSpeed = Constants.clamp(omegaSpeed, -1, 1);

        if (DriveConstants.TRIGGER_BOOST) {
            xSpeed /= 2;
            xSpeed += xSpeed * boost;
            ySpeed /= 2;
            ySpeed += ySpeed * boost;
        }

        xSpeed = xLimiter.calculate(xSpeed * DriveConstants.MAX_SPEED_MPS);
        ySpeed = yLimiter.calculate(ySpeed * DriveConstants.MAX_SPEED_MPS);
        double omegaCommand = turningLimiter.calculate(omegaSpeed * DriveConstants.MAX_ANGULAR_SPEED_MPS);

        if (!Robot.isReal()) {
            odometry.updateGyro(omegaCommand);
        }

        ChassisSpeeds chassisSpeeds;
        if (DriveConstants.FIELD_ORIENTED) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaCommand, odometry.getHeading());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaCommand);
        }

        chassisSpeedsPublisher.set(chassisSpeeds);
        SwerveModuleState[] states = DriveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        swerveStateJoystickPublisher.set(states);
        swerve.setDesiredStates(states);
    }
}