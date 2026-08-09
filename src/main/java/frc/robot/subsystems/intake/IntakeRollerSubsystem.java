package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import frc.robot.Constants.MOTORS;




public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkMax intakeRollerMotor;

    public IntakeRollerSubsystem() {
        System.out.println("Starting Intake Roller Mechanism");

        intakeRollerMotor = new SparkMax(MOTORS.INTAKE_ROLLER.CAN_ID, MotorType.kBrushless);
        var intakeRollerConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.INTAKE_ROLLER.REVERSED);
        intakeRollerMotor.configure(intakeRollerConfig, null, PersistMode.kPersistParameters);
    }

    public void stop() {
        intakeRollerMotor.stopMotor();
    }

    public void start(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        intakeRollerMotor.set(clampedSpeed);
    }
}

