package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final IntakeRollerIO io;
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public IntakeRollerSubsystem(IntakeRollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Roller", inputs);
    }

    public void stop() {
        io.stop();
    }

    public void start(double speed) {
        io.set(speed);
    }
}
