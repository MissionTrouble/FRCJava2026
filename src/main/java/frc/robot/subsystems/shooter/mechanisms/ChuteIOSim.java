package frc.robot.subsystems.shooter.mechanisms;

/** No physics model yet — just reflects the commanded speed back as the applied output. */
public class ChuteIOSim implements ChuteIO {
    private double appliedOutput = 0.0;

    @Override
    public void updateInputs(ChuteIOInputs inputs) {
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
