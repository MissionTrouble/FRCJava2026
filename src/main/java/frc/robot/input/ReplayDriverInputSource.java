package frc.robot.input;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;

/** Plays back a recorded list of samples in real time, driven by wall-clock elapsed time. */
public class ReplayDriverInputSource implements DriverInputSource {
    private static final DriverInputSample NEUTRAL =
        new DriverInputSample(0, 0, 0, 0, 0, false, false, false, false, false, false, false, false);

    private final List<DriverInputSample> samples;
    private final double startTime;
    private int index = 0;
    private DriverInputSample current;

    public ReplayDriverInputSource(List<DriverInputSample> samples) {
        this.samples = samples;
        this.startTime = Timer.getFPGATimestamp();
        this.current = samples.isEmpty() ? NEUTRAL : samples.get(0);
    }

    public boolean isFinished() {
        return index >= samples.size();
    }

    /** Call once per loop to advance playback to the current elapsed time. */
    public void update() {
        double elapsed = Timer.getFPGATimestamp() - startTime;
        while (index < samples.size() && samples.get(index).timestampSeconds() <= elapsed) {
            current = samples.get(index);
            index++;
        }
        if (isFinished()) {
            current = NEUTRAL;
        }
    }

    @Override
    public double getLeftY() {
        return current.leftY();
    }

    @Override
    public double getLeftX() {
        return current.leftX();
    }

    @Override
    public double getRightX() {
        return current.rightX();
    }

    @Override
    public double getRightTrigger() {
        return current.rightTrigger();
    }

    @Override
    public boolean getXButton() {
        return current.xButton();
    }

    @Override
    public boolean getYButton() {
        return current.yButton();
    }

    @Override
    public boolean getStartButton() {
        return current.startButton();
    }

    @Override
    public boolean getBackButton() {
        return current.backButton();
    }

    @Override
    public boolean getLeftBumper() {
        return current.leftBumper();
    }

    @Override
    public boolean getRightBumper() {
        return current.rightBumper();
    }

    @Override
    public boolean getPovUp() {
        return current.povUp();
    }

    @Override
    public boolean getPovDown() {
        return current.povDown();
    }
}
