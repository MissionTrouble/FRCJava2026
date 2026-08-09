package frc.robot.subsystems.shooter.mechanisms;

import org.littletonrobotics.junction.Logger;

public class Chute {
    private final ChuteIO io;
    private final ChuteIOInputsAutoLogged inputs = new ChuteIOInputsAutoLogged();

    public Chute(ChuteIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Chute", inputs);
    }

    public void stop() {
        io.stop();
    }

    public void start(double speed) {
        io.set(speed);
    }
}
