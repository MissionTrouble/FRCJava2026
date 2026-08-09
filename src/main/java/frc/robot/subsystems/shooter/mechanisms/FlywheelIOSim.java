package frc.robot.subsystems.shooter.mechanisms;

import edu.wpi.first.wpilibj.Timer;

/**
 * No real flywheel physics yet — approaches the commanded setpoint with a simple first-order
 * response so atSpeed()-gated commands (e.g. ShooterSubsystem.shoot()) behave sensibly in sim.
 */
public class FlywheelIOSim implements FlywheelIO {
    private static final double RESPONSE_PER_SECOND = 4.0;

    private double velocityRPM = 0.0;
    private double setpointRPM = 0.0;
    private double lastTime = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        double alpha = Math.min(1.0, RESPONSE_PER_SECOND * dt);
        velocityRPM += (setpointRPM - velocityRPM) * alpha;

        inputs.velocityRPM = velocityRPM;
        inputs.setpointRPM = setpointRPM;
    }

    @Override
    public void setVelocity(double rpm) {
        setpointRPM = rpm;
    }

    @Override
    public void stop() {
        setpointRPM = 0.0;
    }
}
