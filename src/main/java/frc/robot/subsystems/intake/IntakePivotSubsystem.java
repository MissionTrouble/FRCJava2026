package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_INFO;
import frc.robot.Constants.REVERSED_MOTORS;

import com.revrobotics.spark.SparkLowLevel.MotorType;




public class IntakePivotSubsystem extends SubsystemBase {
    private final SparkMax intakePivotMotor;

    public IntakePivotSubsystem() {
        System.out.println("Starting Intake Pivot Mechanism");

        intakePivotMotor = new SparkMax(CAN_INFO.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
        var intakePivotConfig = Constants.getDefaultMotorConfig().inverted(REVERSED_MOTORS.INTAKE_PIVOT_MOTOR_REVERSED);
        intakePivotMotor.configure(intakePivotConfig, null, PersistMode.kPersistParameters);
    }

    public void stop() {
        intakePivotMotor.stopMotor();
    }

    public void pivotUp(double speed) {
        double clampedSpeed = Constants.clamp(speed, 0, 1);
        intakePivotMotor.set(clampedSpeed);
    }

    public void pivotDown(double speed) {
        double clampedSpeed = Constants.clamp(speed, 0, 1);
        intakePivotMotor.set(-clampedSpeed);
    }

    public boolean atUpperLimit() {
        return intakePivotMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean atLowerLimit() {
        return intakePivotMotor.getReverseLimitSwitch().isPressed();
    }
}