package frc.robot.subsystems.intake.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem;

public class ToggleIntakePivotCmd extends Command{

    private boolean intakeUp = true;
    private double startTime;
    private double speed;
    private double timeout;
    private IntakePivotSubsystem pivot;


    public ToggleIntakePivotCmd(IntakePivotSubsystem pivot,double speed, double timeout){
        this.pivot=pivot;
        this.speed=speed;
        this.timeout=timeout;
        addRequirements(pivot);
    }

        @Override
        public String getName() {
            return "Toggle Intake Pivot";
        }

        @Override
        public void initialize() {
            startTime = Timer.getFPGATimestamp();

            if (intakeUp) {
                pivot.pivotDown(speed);
            } else {
                pivot.pivotUp(speed);
            }
        }

        @Override
        public boolean isFinished() {
            double elapsed = Timer.getFPGATimestamp() - startTime;

            if (intakeUp) {
                return elapsed >= timeout || pivot.atLowerLimit();
            } else {
                return elapsed >= timeout || pivot.atUpperLimit();
            }
        }

        @Override
        public void end(boolean interrupted) {
            pivot.stop();

            if (!interrupted) {
                intakeUp = !intakeUp;
            }
        }
}
