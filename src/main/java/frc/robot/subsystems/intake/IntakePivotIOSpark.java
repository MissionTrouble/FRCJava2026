package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;

public class IntakePivotIOSpark implements IntakePivotIO {
    private final SparkMax intakePivotMotor;

    public IntakePivotIOSpark() {
        intakePivotMotor = new SparkMax(MOTORS.INTAKE_PIVOT.CAN_ID, MotorType.kBrushless);
        var intakePivotConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.INTAKE_PIVOT.REVERSED);
        intakePivotMotor.configure(intakePivotConfig, null, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.appliedOutput = intakePivotMotor.get();
        inputs.atUpperLimit = intakePivotMotor.getForwardLimitSwitch().isPressed();
        inputs.atLowerLimit = intakePivotMotor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public void set(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        intakePivotMotor.set(clampedSpeed);
    }

    @Override
    public void stop() {
        intakePivotMotor.stopMotor();
    }
}
