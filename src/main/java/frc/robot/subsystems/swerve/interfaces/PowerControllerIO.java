package frc.robot.subsystems.swerve.interfaces;

public interface PowerControllerIO {
    default void run(double speed){};

    default void run(double speed, double dt) {
        run(speed);
    }
    void stop();
    double getDistance();
    double getSpeed();
    void log(String key);

}
