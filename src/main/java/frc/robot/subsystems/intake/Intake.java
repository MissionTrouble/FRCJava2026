package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Intake {
    public final IntakePivotSubsystem pivot;
    public final IntakeRollerSubsystem roller;
    private boolean intakeUp = true;

    public Intake() {
        pivot = new IntakePivotSubsystem();
        roller = new IntakeRollerSubsystem();
        pivot.setDefaultCommand(stopPivot());
        roller.setDefaultCommand(stopRoller());
    }

    public Command joltIntake(double speed, double timeout) {
        return Commands.sequence(
            togglePivot(speed, timeout),
            togglePivot(speed, timeout)
        ).withName("Jolt Intake");
    }

    public Command stopPivot() {
        return Commands.runOnce(pivot::stop, pivot).withName("Stop Intake Pivot");
    }

    public Command togglePivot(double speed, double timeout) {
        return Commands.defer(() -> {
            double startTime = Timer.getFPGATimestamp();
            if (intakeUp) {
                intakeUp = false;
                return Commands.run(() -> pivot.pivotDown(speed), pivot)
                    .until(() -> (Timer.getFPGATimestamp() - startTime) < timeout || pivot.atLowerLimit())
                    .andThen(Commands.runOnce(pivot::stop, pivot));
            } else {
                intakeUp = true;
                return Commands.run(() -> pivot.pivotUp(speed), pivot)
                    .until(() -> (Timer.getFPGATimestamp() - startTime) < timeout || pivot.atUpperLimit())
                    .andThen(Commands.runOnce(pivot::stop, pivot));
            }
        }, java.util.Set.of(pivot)).withName("Toggle Intake Pivot");
    }

    public Command startRoller(double speed) {
        return Commands.runOnce(() -> roller.start(speed), roller).withName("Start Intake Roller");
    }

    public Command stopRoller() {
        return Commands.runOnce(roller::stop, roller).withName("Stop Intake Roller");
    }

    public Command reverseRoller(double speed) {
        return Commands.runOnce(() -> roller.start(-speed), roller).withName("Reverse Intake Roller");
    }
}