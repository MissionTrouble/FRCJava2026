package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

import frc.robot.Constants.CAN_INFO;
import frc.robot.Constants.REVERSED_MOTORS;




public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkMax intakeRollerMotor;

    public IntakeRollerSubsystem() {
        System.out.println("Starting Intake Roller Mechanism");

        intakeRollerMotor = new SparkMax(CAN_INFO.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
        var intakeRollerConfig = Constants.getDefaultMotorConfig().inverted(REVERSED_MOTORS.INTAKE_ROLLER_MOTOR_REVERSED);
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

