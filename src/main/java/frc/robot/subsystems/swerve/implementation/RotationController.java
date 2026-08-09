package frc.robot.subsystems.swerve.implementation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.subsystems.swerve.interfaces.RotationControllerIO;

public class RotationController implements RotationControllerIO{
    private final SparkMax swerveRotationMotor;
    private final SparkClosedLoopController swerveRotationController;
    private final Double offset;
    public RotationController(SparkMax motor,double offset){
        this.offset = offset;
        swerveRotationMotor = motor;
        swerveRotationController = swerveRotationMotor.getClosedLoopController();
    }

    public Rotation2d getRawAngle(){
        return new Rotation2d(swerveRotationMotor.getAnalog().getPosition());
    }

    public Rotation2d getCompensatedAngle(){
        return new Rotation2d(swerveRotationMotor.getAnalog().getPosition()-offset);
    }
    public double getOffset(){
        return offset;
    }


    public void setAngle(double setpoint){
        swerveRotationController.setSetpoint(setpoint-offset, ControlType.kPosition);
    }
    
    public void stop(){
        swerveRotationMotor.set(0);
    }

    public void log(String key){
        SmartDashboard.putNumber(key+" Rotation", swerveRotationController.getSetpoint());

    }
}
