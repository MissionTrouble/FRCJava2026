package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;

/**
 * The seam a future simulated-camera implementation (e.g. MapleSim) slots into: everything below
 * VisionIOPhotonVision's shared PhotonCamera/PhotonPoseEstimator logic stays untouched, only the
 * camera feed itself is simulated.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final Supplier<Pose2d> currentPose;

    public VisionIOPhotonVisionSim(Supplier<Pose2d> currentPose) {
        super();
        this.currentPose = currentPose;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark));

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, Constants.CAMERA_TO_ROBOT.inverse());
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        super.updateInputs(inputs);

        if (Constants.VISION_SIM) {
            if (inputs.poses.length > 0) {
                visionSim.getDebugField().getObject("VisionEstimation")
                        .setPose(inputs.poses[inputs.poses.length - 1].toPose2d());
            } else {
                visionSim.getDebugField().getObject("VisionEstimation").setPoses();
            }
            visionSim.update(currentPose.get());
        }
    }
}
