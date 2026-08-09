package frc.robot.subsystems.swerve.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface RotationControllerIO {
    public Rotation2d getRawAngle();
    public Rotation2d getCompensatedAngle();
    public void setAngle(double setpoint);
    public void log(String key);
    public void stop();
}
