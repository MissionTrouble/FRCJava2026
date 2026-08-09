package frc.robot.input;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

/** Samples a live DriverInputSource once per loop while recording is active. */
public class InputRecorder {
    private final DriverInputSource source;
    private final List<DriverInputSample> samples = new ArrayList<>();
    private boolean recording = false;
    private double startTime;

    public InputRecorder(DriverInputSource source) {
        this.source = source;
    }

    public void start() {
        samples.clear();
        startTime = Timer.getFPGATimestamp();
        recording = true;
    }

    public void stop() {
        recording = false;
    }

    public boolean isRecording() {
        return recording;
    }

    /** Call once per loop; no-op while not recording. */
    public void periodic() {
        if (!recording) {
            return;
        }
        double elapsed = Timer.getFPGATimestamp() - startTime;
        samples.add(new DriverInputSample(
            elapsed,
            source.getLeftY(),
            source.getLeftX(),
            source.getRightX(),
            source.getRightTrigger(),
            source.getXButton(),
            source.getYButton(),
            source.getStartButton(),
            source.getBackButton(),
            source.getLeftBumper(),
            source.getRightBumper(),
            source.getPovUp(),
            source.getPovDown()));
    }

    public List<DriverInputSample> getSamples() {
        return List.copyOf(samples);
    }
}
