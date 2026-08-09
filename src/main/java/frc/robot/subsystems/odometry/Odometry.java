package frc.robot.subsystems.odometry;




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
import frc.robot.Robot;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Odometry {
    private final SwerveDrivePoseEstimator estimator;
    private final GyroIO gyro;

    public Odometry() {
        if (Robot.isReal()) {
            gyro = new Gyro();
        } else {
            gyro = new GyroSim();
        }
        estimator =  new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_DRIVE_KINEMATICS,
        new Rotation2d(),
        DriveConstants.EMPTY_SWERVE_MODULE_POSITIONS,
        new Pose2d());
    }

    public void updateWithModules(SwerveModulePosition[] modulePositions) {
        estimator.update(gyro.getRotation2d(), modulePositions);
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds,Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(pose, timestampSeconds,stdDevs);
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

    public void resetPose(SwerveSubsystem swerve, Vision vision) {
        // Optional<EstimatedRobotPose> pose = vision.getEstimatedPose();
        // if (pose.isEmpty()) {
        //     return;
        // }
        // estimator.resetPosition(gyro.getRotation2d(), swerve.getModulePositions(), pose.get().estimatedPose.toPose2d());
    }

    public Command resetOdometry(SwerveSubsystem swerve, Vision vision) {
        return Commands.runOnce(() -> resetPose(swerve, vision)).withName("resetOdometry");
    }

    public Command resetGyro() {
        return Commands.runOnce(gyro::reset).withName("resetGyro");
    }
}