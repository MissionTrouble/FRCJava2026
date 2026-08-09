package frc.robot.subsystems.swerve.implementation;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.interfaces.PowerControllerIO;


public class PowerController implements PowerControllerIO{
    private final SparkMax swervePowerMotor;
    private final RelativeEncoder encoder;
    public PowerController(SparkMax motor){
        swervePowerMotor = motor;
        encoder = swervePowerMotor.getEncoder();
    }

    @Override
    public void run(double speed){
        double clampedSpeed = Constants.clamp(speed,-1,1);
        swervePowerMotor.set(clampedSpeed);
    }

    @Override
    public void stop(){
        swervePowerMotor.set(0);
    }

    @Override
    public double getSpeed(){
        return swervePowerMotor.get();
    }

    @Override
    public double getDistance(){
        return encoder.getPosition();
    }
    @Override
    public void log(String key){
        SmartDashboard.putNumber(key+" Rotation", encoder.getPosition());

    }
}
