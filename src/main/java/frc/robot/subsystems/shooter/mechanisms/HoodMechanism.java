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

public class HoodMechanism {

    private final SparkMax hoodMotor;
    private final SparkClosedLoopController hoodController;

    private final double hoodOffset;

    public HoodMechanism(double hoodOffset) {
        System.out.println("Starting Hood with hoodOffset: " + hoodOffset);

        this.hoodOffset = hoodOffset;

        hoodMotor = new SparkMax(
                MOTORS.HOOD.CAN_ID,
                MotorType.kBrushless
        );

        SparkBaseConfig hoodConfig =
                Constants.getDefaultMotorConfig()
                        .inverted(MOTORS.HOOD.REVERSED);

        hoodMotor.configure(
                hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        hoodController = hoodMotor.getClosedLoopController();
    }



    public void stop() {
        hoodMotor.stopMotor();
    }

    public void set(double setpoint) {
        hoodController.setSetpoint(
                setpoint - hoodOffset,
                ControlType.kPosition
        );
    }
}
