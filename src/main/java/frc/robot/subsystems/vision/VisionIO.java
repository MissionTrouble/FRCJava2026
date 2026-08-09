package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Raw per-tag-observation IO: exposes the same PhotonVision pose estimates
 * (org.photonvision.EstimatedRobotPose) as parallel arrays instead of a single fused pose, so a
 * future simulated-camera implementation (e.g. MapleSim) can feed the exact same pose-estimation
 * consumer without redesigning this interface.
 */
public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public Pose3d[] poses = new Pose3d[0];
        public double[] timestamps = new double[0];
        public int[] tagCounts = new int[0];
        public double[] averageTagDistances = new double[0];
        public int[] tagIds = new int[0];
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
