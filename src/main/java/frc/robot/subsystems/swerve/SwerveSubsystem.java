package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_INFO;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderOffsets;
import frc.robot.Constants.REVERSED_MOTORS;
import frc.robot.subsystems.swerve.implementation.SwerveModule;
import frc.robot.subsystems.odometry.Odometry;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class SwerveSubsystem extends SubsystemBase{
private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Odometry odometry;

    private final StructArrayTopic<SwerveModuleState> swerveStateTopic =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SmartDashboard/SwerveVelocity", SwerveModuleState.struct);

    private final StructTopic<ChassisSpeeds> chassisSpeedsTopic = NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/ChassisVelocityActual", ChassisSpeeds.struct);

    private final StructArrayPublisher<SwerveModuleState> swerveStatePublisher = swerveStateTopic.publish();
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = chassisSpeedsTopic.publish();
    private final StructTopic<Pose2d> poseTopic = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/RobotPose", Pose2d.struct);

    private final StructPublisher<Pose2d> posePublisher = poseTopic.publish();

    public SwerveSubsystem(Odometry odometry) {
        this.odometry = odometry;
        SparkMax frontLeftPower = new SparkMax(CAN_INFO.FRONT_LEFT_SWERVE_POWER_MOTOR_ID, MotorType.kBrushless);
        frontLeftPower.configure(Constants.getSwerveDriveMotorConfig().inverted(REVERSED_MOTORS.FRONT_LEFT_SWERVE_POWER_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax frontLeftRotation = new SparkMax(CAN_INFO.FRONT_LEFT_SWERVE_ROTATION_MOTOR_ID, MotorType.kBrushless);
        frontLeftRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(REVERSED_MOTORS.FRONT_LEFT_SWERVE_ROTATION_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax frontRightPower = new SparkMax(CAN_INFO.FRONT_RIGHT_SWERVE_POWER_MOTOR_ID, MotorType.kBrushless);
        frontRightPower.configure(Constants.getSwerveDriveMotorConfig().inverted(REVERSED_MOTORS.FRONT_RIGHT_SWERVE_POWER_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax frontRightRotation = new SparkMax(CAN_INFO.FRONT_RIGHT_SWERVE_ROTATION_MOTOR_ID, MotorType.kBrushless);
        frontRightRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(REVERSED_MOTORS.FRONT_RIGHT_SWERVE_ROTATION_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backLeftPower = new SparkMax(CAN_INFO.BACK_LEFT_SWERVE_POWER_MOTOR_ID, MotorType.kBrushless);
        backLeftPower.configure(Constants.getSwerveDriveMotorConfig().inverted(REVERSED_MOTORS.BACK_LEFT_SWERVE_POWER_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backLeftRotation = new SparkMax(CAN_INFO.BACK_LEFT_SWERVE_ROTATION_MOTOR_ID, MotorType.kBrushless);
        backLeftRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(REVERSED_MOTORS.BACK_LEFT_SWERVE_ROTATION_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backRightPower = new SparkMax(CAN_INFO.BACK_RIGHT_SWERVE_POWER_MOTOR_ID, MotorType.kBrushless);
        backRightPower.configure(Constants.getSwerveDriveMotorConfig().inverted(REVERSED_MOTORS.BACK_RIGHT_SWERVE_POWER_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMax backRightRotation = new SparkMax(CAN_INFO.BACK_RIGHT_SWERVE_ROTATION_MOTOR_ID, MotorType.kBrushless);
        backRightRotation.configure(Constants.getSwerveRotationMotorConfig().inverted(REVERSED_MOTORS.BACK_RIGHT_SWERVE_ROTATION_MOTOR_REVERSED), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        frontLeft = new SwerveModule("FrontLeft", frontLeftPower, frontLeftRotation, EncoderOffsets.FRONT_LEFT_ENCODER_OFFSET);
        frontRight = new SwerveModule("FrontRight", frontRightPower, frontRightRotation, EncoderOffsets.FRONT_RIGHT_ENCODER_OFFSET);
        backLeft = new SwerveModule("BackLeft", backLeftPower, backLeftRotation, EncoderOffsets.BACK_LEFT_ENCODER_OFFSET);
        backRight = new SwerveModule("BackRight", backRightPower, backRightRotation, EncoderOffsets.BACK_RIGHT_ENCODER_OFFSET);
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED_MPS);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
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
        swerveStatePublisher.set(states);

        chassisSpeedsPublisher.set(Constants.DriveConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(states));

        posePublisher.set(getOdometryPose());
    }

    @Override
    public void periodic() {
        updateOdometry();
        log();
    }
}
