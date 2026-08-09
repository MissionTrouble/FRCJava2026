package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;

public class IntakeRollerIOSpark implements IntakeRollerIO {
    private final SparkMax intakeRollerMotor;

    public IntakeRollerIOSpark() {
        intakeRollerMotor = new SparkMax(MOTORS.INTAKE_ROLLER.CAN_ID, MotorType.kBrushless);
        var intakeRollerConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.INTAKE_ROLLER.REVERSED);
        intakeRollerMotor.configure(intakeRollerConfig, null, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.appliedOutput = intakeRollerMotor.get();
    }

    @Override
    public void set(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        intakeRollerMotor.set(clampedSpeed);
    }

    @Override
    public void stop() {
        intakeRollerMotor.stopMotor();
    }
}
