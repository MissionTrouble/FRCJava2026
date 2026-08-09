package frc.robot.subsystems.shooter;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.shooter.mechanisms.Chute;
import frc.robot.subsystems.shooter.mechanisms.ChuteIO;
import frc.robot.subsystems.shooter.mechanisms.Flywheel;
import frc.robot.subsystems.shooter.mechanisms.FlywheelIO;
import frc.robot.subsystems.shooter.mechanisms.Hood;
import frc.robot.subsystems.shooter.mechanisms.HoodIO;

public class ShooterSubsystem extends SubsystemBase{

    public final Chute chute;
    public final Flywheel flywheel;
    public final Hood hood;
    private final Odometry odometry;

    private final InterpolatingDoubleTreeMap hoodCalibration =
            new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap flywheelCalibration =
            new InterpolatingDoubleTreeMap();

    public ShooterSubsystem(Odometry odometry, ChuteIO chuteIO, FlywheelIO flywheelIO, HoodIO hoodIO) {
        this.odometry = odometry;
        hoodCalibration.put(1.0, 0.2);
        hoodCalibration.put(2.0, 0.3);
        hoodCalibration.put(3.0, 0.6);
        flywheelCalibration.put(1.0, 2700.0);
        flywheelCalibration.put(2.0, 2900.0);
        flywheelCalibration.put(3.0, 3100.0);
        chute = new Chute(chuteIO);
        flywheel = new Flywheel(flywheelIO);
        hood = new Hood(hoodIO);

    }

    @Override
    public void periodic() {
        chute.updateInputs();
        flywheel.updateInputs();
        hood.updateInputs();
    }

    public Command shoot(double chuteSpeed, double flywheelError) {
        Command shoot = new Command() {

            @Override
            public String getName() {
                return "Shoot";
            }

            @Override
            public void initialize() {
                Pose2d estimatedPose = odometry.getPose();
                double distance = Constants.HUB_LOCATION.getDistance(estimatedPose.getTranslation());
                hood.set(hoodCalibration.get(distance));
                flywheel.set(flywheelCalibration.get(distance));
            }

            @Override
            public boolean isFinished() {
                return flywheel.atSpeed(flywheelError);
            }

            @Override
            public void execute() {
                    if(flywheel.atSpeed(flywheelError)){
                        chute.start(chuteSpeed);
                    }
            }

            @Override
            public void end(boolean interrupted) {
                chute.stop();
                flywheel.stop();
                hood.stop();
                
            }
        };
        shoot.addRequirements(this);
        return shoot;
    }

    public Command stop() {
        Command stop = new Command() {

            @Override
            public String getName() {
                return "Stop";
            }

            @Override
            public void initialize() {
                chute.stop();
                hood.stop();
                flywheel.stop();
            }
        };
        stop.addRequirements(this);
        return stop;
    }
}
