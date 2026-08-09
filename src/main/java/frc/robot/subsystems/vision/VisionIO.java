package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.odometry.Odometry;

public interface VisionIO {
    void updatePose(Odometry odometry);
    Optional<EstimatedRobotPose> getEstimatedPose();
}