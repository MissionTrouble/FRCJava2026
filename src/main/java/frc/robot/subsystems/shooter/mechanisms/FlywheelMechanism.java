package frc.robot.subsystems.shooter.mechanisms;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;

public class FlywheelMechanism {


    private final SparkMax flywheelMotor;
    private final SparkClosedLoopController flywheelController;

    public FlywheelMechanism() {
        System.out.println("Starting Flywheel Mechanism");

        flywheelMotor = new SparkMax(
                MOTORS.FLYWHEEL.CAN_ID,
                MotorType.kBrushless
        );

        SparkBaseConfig flywheelConfig =
                Constants.getDefaultMotorConfig()
                        .inverted(MOTORS.FLYWHEEL.REVERSED);

        flywheelMotor.configure(
                flywheelConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        flywheelController = flywheelMotor.getClosedLoopController();
    }



    public void stop() {
        flywheelMotor.stopMotor();
    }

    public void set(double speed) {
        flywheelController.setSetpoint(
                speed,
                ControlType.kVelocity
        );
    }

    public boolean atSpeed(double error) {
        double goal = flywheelController.getSetpoint();
        double speed = flywheelMotor.getEncoder().getVelocity();

        return goal * (1 - error) < speed
                && speed < goal * (1 + error);
    }
}
