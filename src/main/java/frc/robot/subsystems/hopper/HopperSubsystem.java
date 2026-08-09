package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    private void stop() {
        io.stop();
    }

    private void start(double speed) {
        io.set(speed);
    }

    public double getSpeed() {
        return inputs.appliedOutput;
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("Stop Hopper");
    }

    public Command startCommand(double speed) {
        return Commands.runOnce(() -> this.start(speed), this).withName("Start Hopper");
    }

    public Command reverseCommand(double speed) {
        return Commands.runOnce(() -> this.start(-speed), this).withName("Reverse Hopper");
    }
}
