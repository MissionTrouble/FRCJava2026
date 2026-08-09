package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.odometry.Odometry;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final Odometry odometry;

    public Vision(VisionIO io, Odometry odometry) {
        this.io = io;
        this.odometry = odometry;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        for (int i = 0; i < inputs.poses.length; i++) {
            Matrix<N3, N1> stdDevs = getEstimationStdDevs(inputs.tagCounts[i], inputs.averageTagDistances[i]);
            odometry.addVisionMeasurement(inputs.poses[i].toPose2d(), inputs.timestamps[i], stdDevs);
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(int tagCount, double averageTagDistance) {
        if (tagCount == 0) {
            return Constants.kSingleTagStdDevs;
        }

        Matrix<N3, N1> estStdDevs = tagCount > 1 ? Constants.kMultiTagStdDevs : Constants.kSingleTagStdDevs;
        if (tagCount == 1 && averageTagDistance > 4) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        return estStdDevs.times(1 + (averageTagDistance * averageTagDistance / 30));
    }
}
