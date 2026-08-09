package frc.robot.subsystems.shooter.mechanisms;

import org.littletonrobotics.junction.Logger;

public class Hood {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Hood", inputs);
    }

    public void stop() {
        io.stop();
    }

    public void set(double setpoint) {
        io.setPosition(setpoint);
    }
}
