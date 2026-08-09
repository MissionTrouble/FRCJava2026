package frc.robot.subsystems.swerve.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.interfaces.RotationControllerIO;

public class RotationControllerSim implements RotationControllerIO{
    private final SparkMaxSim swerveRotationMotor;
    private final SparkAbsoluteEncoderSim swerveRotationEncoder;

    public RotationControllerSim(SparkMax motor){
        swerveRotationMotor = new SparkMaxSim(motor, Constants.getNeoSimMotor());
        swerveRotationEncoder = swerveRotationMotor.getAbsoluteEncoderSim();
    }

    public Rotation2d getRawAngle(){
        return new Rotation2d(swerveRotationEncoder.getPosition());
    }
    
    public Rotation2d getCompensatedAngle(){
        return new Rotation2d(swerveRotationEncoder.getPosition());
    }

    public void setAngle(double setpoint){
        swerveRotationEncoder.setPosition(setpoint);
    }

    public void stop(){}
    public void log(String key){
        SmartDashboard.putNumber(key+" Rotation", swerveRotationEncoder.getPosition());

    }
}
