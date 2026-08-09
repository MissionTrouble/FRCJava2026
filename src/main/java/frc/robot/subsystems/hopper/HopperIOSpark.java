package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;

public class HopperIOSpark implements HopperIO {
    private final SparkMax hopperMotor;

    public HopperIOSpark() {
        hopperMotor = new SparkMax(MOTORS.HOPPER.CAN_ID, MotorType.kBrushless);
        var hopperConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.HOPPER.REVERSED);
        hopperMotor.configure(hopperConfig, null, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.appliedOutput = hopperMotor.get();
    }

    @Override
    public void set(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        hopperMotor.set(clampedSpeed);
    }

    @Override
    public void stop() {
        hopperMotor.stopMotor();
    }
}
