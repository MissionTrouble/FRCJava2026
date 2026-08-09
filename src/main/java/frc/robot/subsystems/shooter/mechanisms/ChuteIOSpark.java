package frc.robot.subsystems.shooter.mechanisms;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;

public class ChuteIOSpark implements ChuteIO {
    private final SparkMax chuteMotor;

    public ChuteIOSpark() {
        chuteMotor = new SparkMax(MOTORS.CHUTE.CAN_ID, MotorType.kBrushless);
        var chuteConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.CHUTE.REVERSED);
        chuteMotor.configure(chuteConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ChuteIOInputs inputs) {
        inputs.appliedOutput = chuteMotor.get();
    }

    @Override
    public void set(double speed) {
        chuteMotor.set(speed);
    }

    @Override
    public void stop() {
        chuteMotor.stopMotor();
    }
}
