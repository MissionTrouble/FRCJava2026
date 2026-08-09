package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
    private final IntakePivotIO io;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    public IntakePivotSubsystem(IntakePivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Pivot", inputs);
    }

    public void stop() {
        io.stop();
    }

    public void pivotUp(double speed) {
        io.set(speed);
    }

    public void pivotDown(double speed) {
        io.set(-speed);
    }

    public boolean atUpperLimit() {
        return inputs.atUpperLimit;
    }

    public boolean atLowerLimit() {
        return inputs.atLowerLimit;
    }
}
