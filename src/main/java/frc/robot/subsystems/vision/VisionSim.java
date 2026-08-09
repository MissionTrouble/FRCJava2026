package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;
import frc.robot.subsystems.odometry.Odometry;

public class VisionSim implements VisionIO {

    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;

    public VisionSim(String cameraName) {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProps.setFPS(20);
        cameraProps.setAvgLatencyMs(20);

        PhotonCamera camera = new PhotonCamera(cameraName);
        cameraSim = new PhotonCameraSim(camera, cameraProps);
        visionSim.addCamera(cameraSim, Constants.CAMERA_TO_ROBOT.inverse());

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.CAMERA_TO_ROBOT);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updatePose(Odometry odometry) {
        if (!Constants.VISION_SIM) {
            return;
        }
        Pose2d currentPose = odometry.getPose();
        visionSim.update(currentPose);

        Optional<EstimatedRobotPose> estimate = getEstimatedPose();
        if (estimate.isPresent()) {
            EstimatedRobotPose est = estimate.get();
            odometry.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        var results = cameraSim.getCamera().getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }

        var latest = results.get(results.size() - 1);
        if (!latest.hasTargets()) {
            return Optional.empty();
        }

        return poseEstimator.update(latest);
    }
}