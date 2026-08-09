package frc.robot.subsystems.shooter.mechanisms;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;

public class FlywheelIOSpark implements FlywheelIO {
    private final SparkMax flywheelMotor;
    private final SparkClosedLoopController flywheelController;

    public FlywheelIOSpark() {
        flywheelMotor = new SparkMax(MOTORS.FLYWHEEL.CAN_ID, MotorType.kBrushless);

        SparkBaseConfig flywheelConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.FLYWHEEL.REVERSED);

        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelController = flywheelMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityRPM = flywheelMotor.getEncoder().getVelocity();
        inputs.setpointRPM = flywheelController.getSetpoint();
    }

    @Override
    public void setVelocity(double rpm) {
        flywheelController.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
    }
}
