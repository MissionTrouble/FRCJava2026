package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;
import frc.robot.subsystems.odometry.Odometry;

public class Vision implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private static final double MAX_TAG_DISTANCE_METERS = 5.0;
    private static final double MAX_AMBIGUITY = 0.2;

    public Vision() {
        camera = new PhotonCamera("visionCamera");
        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.CAMERA_TO_ROBOT);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<Double> getDistanceToHub() {
        Translation2d point = new Translation2d(1, 2);
        return getEstimatedPose()
                .map(est -> est.estimatedPose.toPose2d()
                        .getTranslation()
                        .getDistance(point));
    }

    @Override
    public void updatePose(Odometry odometry) {
        Optional<EstimatedRobotPose> visionPose = getEstimatedPose();
        if (visionPose.isPresent()) {
            EstimatedRobotPose estimate = visionPose.get();
            odometry.addVisionMeasurement(estimate.estimatedPose.toPose2d(), Timer.getFPGATimestamp());
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);

        if (!latest.hasTargets()) {
            return Optional.empty();
        }

        if (!isValidResult(latest)) {
            return Optional.empty();
        }

        return poseEstimator.update(latest);
    }

    private boolean isValidResult(PhotonPipelineResult result) {
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getPoseAmbiguity() > MAX_AMBIGUITY) {
                continue;
            }

            double distance = target.getBestCameraToTarget()
                    .getTranslation()
                    .getNorm();

            if (distance <= MAX_TAG_DISTANCE_METERS) {
                return true;
            }
        }

        return false;
    }
}