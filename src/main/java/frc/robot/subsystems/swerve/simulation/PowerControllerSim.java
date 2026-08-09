package frc.robot.subsystems.swerve.simulation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.interfaces.PowerControllerIO;


public class PowerControllerSim implements PowerControllerIO{
    private final SparkMaxSim swervePowerMotor;
    private final SparkRelativeEncoderSim encoder;

    public PowerControllerSim(SparkMax motor){
        swervePowerMotor = new SparkMaxSim(motor, Constants.getNeoSimMotor());
        encoder = swervePowerMotor.getRelativeEncoderSim();

    }

    @Override
    public void run(double speed,double dt){
        double speedMetersPerSecond = getSpeed();

        double distanceDelta = speedMetersPerSecond * dt;

        encoder.setPosition(encoder.getPosition() + distanceDelta);
        double clampedSpeed = Constants.clamp(speed,-1,1);
        swervePowerMotor.setAppliedOutput(clampedSpeed);
    }
    @Override
    public void stop(){
        swervePowerMotor.setAppliedOutput(0);
    }
    @Override
    public double getDistance(){
        return encoder.getPosition();
    }
    @Override
    public double getSpeed(){
        return swervePowerMotor.getAppliedOutput()*DriveConstants.MAX_SPEED_MPS;
    }
    @Override
    public void log(String key){
        SmartDashboard.putNumber(key+" Power", swervePowerMotor.getAppliedOutput());
    }

}
