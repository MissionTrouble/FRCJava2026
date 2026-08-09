package frc.robot.subsystems.odometry;




import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Odometry {
    private final SwerveDrivePoseEstimator estimator;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    public Odometry(GyroIO gyroIO) {
        this.gyroIO = gyroIO;
        estimator =  new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_DRIVE_KINEMATICS,
        new Rotation2d(),
        DriveConstants.EMPTY_SWERVE_MODULE_POSITIONS,
        new Pose2d());
    }

    public void updateWithModules(SwerveModulePosition[] modulePositions) {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);
        estimator.update(gyroInputs.yawPosition, modulePositions);
        Logger.recordOutput("Odometry/Pose", getPose());
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds,Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(pose, timestampSeconds,stdDevs);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return gyroInputs.yawPosition;
    }

    public void updateGyro(double speed) {
        gyroIO.setOmega(speed);
    }

    public void resetPose(SwerveSubsystem swerve, Vision vision) {
        // Optional<EstimatedRobotPose> pose = vision.getEstimatedPose();
        // if (pose.isEmpty()) {
        //     return;
        // }
        // estimator.resetPosition(gyroInputs.yawPosition, swerve.getModulePositions(), pose.get().estimatedPose.toPose2d());
    }

    public Command resetOdometry(SwerveSubsystem swerve, Vision vision) {
        return Commands.runOnce(() -> resetPose(swerve, vision)).withName("resetOdometry");
    }

    public Command resetGyro() {
        return Commands.runOnce(gyroIO::reset).withName("resetGyro");
    }
}