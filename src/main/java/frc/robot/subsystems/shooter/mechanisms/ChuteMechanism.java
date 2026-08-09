package frc.robot.subsystems.shooter.mechanisms;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.CAN_INFO;
import frc.robot.Constants.REVERSED_MOTORS;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ChuteMechanism extends SubsystemBase {
    private final SparkMax chuteMotor;

    public ChuteMechanism() {
        System.out.println("Starting Chute Mechanism");
        chuteMotor = new SparkMax(CAN_INFO.CHUTE_MOTOR_ID, MotorType.kBrushless);
        var chuteConfig = Constants.getDefaultMotorConfig().inverted(REVERSED_MOTORS.CHUTE_MOTOR_REVERSED);
        chuteMotor.configure(chuteConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stop() {
        chuteMotor.stopMotor();
    }

    public void start(double speed) {
        chuteMotor.set(speed);
    }
}
