package frc.robot.input;

/**
 * Everything RobotContainer reads off the driver controller, abstracted so the same axis
 * suppliers/triggers can be pointed at either the live controller or a recorded playback.
 */
public interface DriverInputSource {
    double getLeftY();

    double getLeftX();

    double getRightX();

    double getRightTrigger();

    boolean getXButton();

    boolean getYButton();

    boolean getStartButton();

    boolean getBackButton();

    boolean getLeftBumper();

    boolean getRightBumper();

    boolean getPovUp();

    boolean getPovDown();
}
