package frc.robot.subsystems.odometry;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIO;

public class Odometry {
    private final SwerveDrivePoseEstimator estimator;
    private final GyroIO gyro;

    public Odometry(GyroIO gyro, SwerveDrivePoseEstimator estimator) {
        this.gyro = gyro;
        this.estimator = estimator;
    }

    public void updateWithModules(SwerveModulePosition[] modulePositions) {
        estimator.update(gyro.getRotation2d(), modulePositions);
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        estimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public void updateGyro(double speed) {
        gyro.update(speed);
    }

    public void resetPose(SwerveSubsystem swerve, VisionIO vision) {
        Optional<EstimatedRobotPose> pose = vision.getEstimatedPose();
        if (pose.isEmpty()) {
            return;
        }
        estimator.resetPosition(gyro.getRotation2d(), swerve.getModulePositions(), pose.get().estimatedPose.toPose2d());
    }

    public Command resetOdometry(SwerveSubsystem swerve, VisionIO vision) {
        return Commands.runOnce(() -> resetPose(swerve, vision)).withName("resetOdometry");
    }

    public Command resetGyro() {
        return Commands.runOnce(gyro::reset).withName("resetGyro");
    }
}