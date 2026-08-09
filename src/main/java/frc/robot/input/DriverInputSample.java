package frc.robot.input;

/** A single timestamped snapshot of driver controller state, relative to the start of a recording. */
public record DriverInputSample(
    double timestampSeconds,
    double leftY,
    double leftX,
    double rightX,
    double rightTrigger,
    boolean xButton,
    boolean yButton,
    boolean startButton,
    boolean backButton,
    boolean leftBumper,
    boolean rightBumper,
    boolean povUp,
    boolean povDown) {}
