package frc.robot.subsystems.intake;

/** No pivot physics/limit-switch model yet — limits are never reported as reached. */
public class IntakePivotIOSim implements IntakePivotIO {
    private double appliedOutput = 0.0;

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.appliedOutput = appliedOutput;
        inputs.atUpperLimit = false;
        inputs.atLowerLimit = false;
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
