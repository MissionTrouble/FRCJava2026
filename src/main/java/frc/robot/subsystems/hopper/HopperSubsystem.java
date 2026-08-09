package frc.robot.subsystems.hopper;

import frc.robot.Constants.CAN_INFO;
import frc.robot.Constants.REVERSED_MOTORS;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class HopperSubsystem extends SubsystemBase {
    private final SparkMax hopperMotor;

    public HopperSubsystem() {
        System.out.println("Starting Hopper Mechanism");

        hopperMotor = new SparkMax(CAN_INFO.HOPPER_MOTOR_ID, MotorType.kBrushless);
        var hopperConfig = Constants.getDefaultMotorConfig().inverted(REVERSED_MOTORS.HOPPER_MOTOR_REVERSED);
        hopperMotor.configure(hopperConfig, null, PersistMode.kPersistParameters);
    }

    private void stop() {
        hopperMotor.stopMotor();
    }

    private void start(double speed) {
        double clampedSpeed = Constants.clamp(speed, -1, 1);
        hopperMotor.set(clampedSpeed);
    }

    public double getSpeed() {
        return hopperMotor.get();
    }


    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("Stop Hopper");
    }

    public Command startCommand(double speed) {
        return Commands.runOnce(() -> this.start(speed), this).withName("Start Hopper");
    }

    public Command reverseCommand(double speed) {
        return Commands.runOnce(() -> this.start(-speed), this).withName("Reverse Hopper");
    }
}
