package frc.robot.subsystems.shooter.mechanisms;

/** Nothing currently gates on hood position, so an instant-setpoint placeholder is sufficient. */
public class HoodIOSim implements HoodIO {
    private final double hoodOffset;
    private double setpointRad = 0.0;

    public HoodIOSim(double hoodOffset) {
        this.hoodOffset = hoodOffset;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRad = setpointRad - hoodOffset;
        inputs.setpointRad = setpointRad - hoodOffset;
    }

    @Override
    public void setPosition(double setpoint) {
        setpointRad = setpoint;
    }

    @Override
    public void stop() {}
}
