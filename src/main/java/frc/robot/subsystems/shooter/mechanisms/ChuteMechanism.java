package frc.robot.subsystems.shooter.mechanisms;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.MOTORS;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ChuteMechanism extends SubsystemBase {
    private final SparkMax chuteMotor;

    public ChuteMechanism() {
        System.out.println("Starting Chute Mechanism");
        chuteMotor = new SparkMax(MOTORS.CHUTE.CAN_ID, MotorType.kBrushless);
        var chuteConfig = Constants.getDefaultMotorConfig().inverted(MOTORS.CHUTE.REVERSED);
        chuteMotor.configure(chuteConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stop() {
        chuteMotor.stopMotor();
    }

    public void start(double speed) {
        chuteMotor.set(speed);
    }
}
