package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.Constants;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator photonEstimator;

    public VisionIOPhotonVision() {
        camera = new PhotonCamera("visionCamera");
        photonEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                Constants.CAMERA_TO_ROBOT.inverse()
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<Pose3d> poses = new ArrayList<>();
        List<Double> timestamps = new ArrayList<>();
        List<Integer> tagCounts = new ArrayList<>();
        List<Double> averageTagDistances = new ArrayList<>();
        List<Integer> tagIds = new ArrayList<>();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            if (visionEst.isEmpty()) {
                continue;
            }

            EstimatedRobotPose est = visionEst.get();
            int numTags = 0;
            double avgDist = 0;
            for (var tgt : result.getTargets()) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                tagIds.add(tgt.getFiducialId());
                avgDist += tagPose.get().toPose2d().getTranslation()
                        .getDistance(est.estimatedPose.toPose2d().getTranslation());
            }
            if (numTags > 0) {
                avgDist /= numTags;
            }

            poses.add(est.estimatedPose);
            timestamps.add(est.timestampSeconds);
            tagCounts.add(numTags);
            averageTagDistances.add(avgDist);
        }

        inputs.poses = poses.toArray(new Pose3d[0]);
        inputs.timestamps = timestamps.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.tagCounts = tagCounts.stream().mapToInt(Integer::intValue).toArray();
        inputs.averageTagDistances = averageTagDistances.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    }
}
