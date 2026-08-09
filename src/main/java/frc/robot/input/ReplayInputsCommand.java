package frc.robot.input;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Redirects a {@link SwitchableDriverInputSource} to a recorded playback for its duration, then
 * restores the live source. Has no requirements: the actual driving happens through whatever
 * default commands/triggers are already bound against the switchable source.
 */
public class ReplayInputsCommand extends Command {
    private final SwitchableDriverInputSource inputs;
    private final DriverInputSource liveInputs;
    private final List<DriverInputSample> samples;
    private ReplayDriverInputSource playback;

    public ReplayInputsCommand(
        SwitchableDriverInputSource inputs, DriverInputSource liveInputs, List<DriverInputSample> samples) {
        this.inputs = inputs;
        this.liveInputs = liveInputs;
        this.samples = samples;
        setName("Replay Recorded Auto");
    }

    @Override
    public void initialize() {
        playback = new ReplayDriverInputSource(samples);
        inputs.setDelegate(playback);
    }

    @Override
    public void execute() {
        playback.update();
    }

    @Override
    public boolean isFinished() {
        return playback.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        inputs.setDelegate(liveInputs);
    }
}
