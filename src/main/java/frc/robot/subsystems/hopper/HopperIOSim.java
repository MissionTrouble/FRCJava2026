package frc.robot.subsystems.hopper;

/** No physics model yet — just reflects the commanded speed back as the applied output. */
public class HopperIOSim implements HopperIO {
    private double appliedOutput = 0.0;

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.appliedOutput = appliedOutput;
    }

    @Override
    public void set(double speed) {
        appliedOutput = speed;
    }

    @Override
    public void stop() {
        appliedOutput = 0.0;
    }
}
