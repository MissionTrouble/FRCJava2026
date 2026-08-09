package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderOffsets;
import frc.robot.Constants.MOTORS;
import frc.robot.Robot;
import frc.robot.subsystems.odometry.Odometry;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class SwerveSubsystem extends SubsystemBase{
private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Odometry odometry;

    public SwerveSubsystem(Odometry odometry) {
        this.odometry = odometry;
        SparkMax frontLeftPower = new SparkMax(MOTORS.FRONT_LEFT_SWERVE_POWER.CAN_ID, MotorType.kBrushless);
        frontLeftPower.configure(Constants.getSwerveDriveMotorConfig().inverted(MOTORS.FRONT_LEFT_SWERVE_POWER.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax frontLeftRotation = new SparkMax(MOTORS.FRONT_LEFT_SWERVE_ROTATION.CAN_ID, MotorType.kBrushless);
        frontLeftRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(MOTORS.FRONT_LEFT_SWERVE_ROTATION.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax frontRightPower = new SparkMax(MOTORS.FRONT_RIGHT_SWERVE_POWER.CAN_ID, MotorType.kBrushless);
        frontRightPower.configure(Constants.getSwerveDriveMotorConfig().inverted(MOTORS.FRONT_RIGHT_SWERVE_POWER.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax frontRightRotation = new SparkMax(MOTORS.FRONT_RIGHT_SWERVE_ROTATION.CAN_ID, MotorType.kBrushless);
        frontRightRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(MOTORS.FRONT_RIGHT_SWERVE_ROTATION.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backLeftPower = new SparkMax(MOTORS.BACK_LEFT_SWERVE_POWER.CAN_ID, MotorType.kBrushless);
        backLeftPower.configure(Constants.getSwerveDriveMotorConfig().inverted(MOTORS.BACK_LEFT_SWERVE_POWER.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backLeftRotation = new SparkMax(MOTORS.BACK_LEFT_SWERVE_ROTATION.CAN_ID, MotorType.kBrushless);
        backLeftRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(MOTORS.BACK_LEFT_SWERVE_ROTATION.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backRightPower = new SparkMax(MOTORS.BACK_RIGHT_SWERVE_POWER.CAN_ID, MotorType.kBrushless);
        backRightPower.configure(Constants.getSwerveDriveMotorConfig().inverted(MOTORS.BACK_RIGHT_SWERVE_POWER.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backRightRotation = new SparkMax(MOTORS.BACK_RIGHT_SWERVE_ROTATION.CAN_ID, MotorType.kBrushless);
        backRightRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(MOTORS.BACK_RIGHT_SWERVE_ROTATION.REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        frontLeft = new SwerveModule("FrontLeft", makePowerController(frontLeftPower), makeRotationController(frontLeftRotation, EncoderOffsets.FRONT_LEFT));
        frontRight = new SwerveModule("FrontRight", makePowerController(frontRightPower), makeRotationController(frontRightRotation, EncoderOffsets.FRONT_RIGHT));
        backLeft = new SwerveModule("BackLeft", makePowerController(backLeftPower), makeRotationController(backLeftRotation, EncoderOffsets.BACK_LEFT));
        backRight = new SwerveModule("BackRight", makePowerController(backRightPower), makeRotationController(backRightRotation, EncoderOffsets.BACK_RIGHT));
    }

    private static PowerControllerIO makePowerController(SparkMax motor) {
        return Robot.isReal() ? new PowerControllerSpark(motor) : new PowerControllerSim(motor);
    }

    private static RotationControllerIO makeRotationController(SparkMax motor, double offset) {
        return Robot.isReal() ? new RotationControllerSpark(motor, offset) : new RotationControllerSim(motor);
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_MPS);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void updateModuleInputs() {
        frontLeft.updateInputs();
        frontRight.updateInputs();
        backLeft.updateInputs();
        backRight.updateInputs();
    }

    public void updateOdometry() {
        odometry.updateWithModules(getModulePositions());
    }

    public Pose2d getOdometryPose() {
        return odometry.getPose();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            backRight.getModuleState()
        };
    }

    public void log() {
        SwerveModuleState[] states = getModuleStates();
        Logger.recordOutput("Swerve/ModuleStates", states);
        Logger.recordOutput("Swerve/ChassisSpeeds", Constants.DriveConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(states));
    }

    @Override
    public void periodic() {
        updateModuleInputs();
        updateOdometry();
        log();
    }
}
