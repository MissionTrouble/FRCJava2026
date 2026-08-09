package frc.robot.subsystems.shooter.mechanisms;

import org.littletonrobotics.junction.Logger;

public class Flywheel {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Flywheel", inputs);
    }

    public void stop() {
        io.stop();
    }

    public void set(double speed) {
        io.setVelocity(speed);
    }

    public boolean atSpeed(double error) {
        double goal = inputs.setpointRPM;
        double speed = inputs.velocityRPM;

        return goal * (1 - error) < speed && speed < goal * (1 + error);
    }
}
