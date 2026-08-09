// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.Joysticks;
import frc.robot.input.DriverInputSample;
import frc.robot.input.InputRecorder;
import frc.robot.input.InputRecording;
import frc.robot.input.LiveDriverInputSource;
import frc.robot.input.ReplayInputsCommand;
import frc.robot.input.SwitchableDriverInputSource;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOReal;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOSpark;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSpark;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.mechanisms.ChuteIOSim;
import frc.robot.subsystems.shooter.mechanisms.ChuteIOSpark;
import frc.robot.subsystems.shooter.mechanisms.FlywheelIOSim;
import frc.robot.subsystems.shooter.mechanisms.FlywheelIOSpark;
import frc.robot.subsystems.shooter.mechanisms.HoodIOSim;
import frc.robot.subsystems.shooter.mechanisms.HoodIOSpark;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private static final String RECORDING_FILE = "auto-recording.csv";

    private final CommandXboxController driverController =
      new CommandXboxController(Joysticks.DRIVER_CONTROLLER_PORT);

    // Everything in configureBindings()/Swerve's axis suppliers reads from `inputs`, so swapping
    // its delegate is enough to make teleop bindings replay a recorded run during auto.
    private final LiveDriverInputSource liveInputs = new LiveDriverInputSource(driverController);
    private final SwitchableDriverInputSource inputs = new SwitchableDriverInputSource(liveInputs);
    private final InputRecorder recorder = new InputRecorder(liveInputs);

  // The robot's subsystems and commands are defined here...
    private final Intake intake;
    private final HopperSubsystem hopper;
    private final ShooterSubsystem shooter;
    private final Vision vision;
    private final Swerve swerve;
    private final Odometry odometry;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    GyroIO gyroIO = Robot.isReal() ? new GyroIOReal() : new GyroIOSim();
    odometry = new Odometry(gyroIO);

    VisionIO visionIO = Robot.isReal()
        ? new VisionIOPhotonVision()
        : (Constants.VISION_SIM ? new VisionIOPhotonVisionSim(odometry::getPose) : new VisionIO() {});
    vision = new Vision(visionIO, odometry);

    swerve = new Swerve(odometry,
      () -> -inputs.getLeftY(),
      () -> -inputs.getLeftX(),
      () -> inputs.getRightX(),
      () -> inputs.getRightTrigger()
    );

    if (Robot.isReal()) {
      intake = new Intake(new IntakePivotIOSpark(), new IntakeRollerIOSpark());
      hopper = new HopperSubsystem(new HopperIOSpark());
      shooter = new ShooterSubsystem(odometry, new ChuteIOSpark(), new FlywheelIOSpark(), new HoodIOSpark(0));
    } else {
      intake = new Intake(new IntakePivotIOSim(), new IntakeRollerIOSim());
      hopper = new HopperSubsystem(new HopperIOSim());
      shooter = new ShooterSubsystem(odometry, new ChuteIOSim(), new FlywheelIOSim(), new HoodIOSim(0));
    }
    intake.pivot.setDefaultCommand(intake.stopPivot());
    intake.roller.setDefaultCommand(intake.stopRoller());
    hopper.setDefaultCommand(hopper.stopCommand());

    swerve.swerve.setDefaultCommand(swerve.joystickDriveCommand());

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link CommandXboxController}.
   */
  private void configureBindings() {
    // Bound against `inputs` (not driverController directly) so a recorded run can be replayed
    // through these exact same bindings during auto.
    new Trigger(inputs::getXButton).toggleOnTrue(swerve.pointToHubCommand());
    new Trigger(inputs::getStartButton).onTrue(odometry.resetGyro());
    new Trigger(inputs::getBackButton).whileTrue(odometry.resetOdometry(swerve.swerve, vision));
    new Trigger(inputs::getPovDown).whileTrue(intake.reverseRoller(0.5));
    new Trigger(inputs::getYButton).toggleOnTrue(intake.startRoller(0.5));
    new Trigger(inputs::getLeftBumper).onTrue(intake.togglePivot(0.5, 0.1));
    new Trigger(inputs::getLeftBumper).toggleOnTrue(hopper.startCommand(0.5));
    new Trigger(inputs::getPovUp).whileTrue(hopper.reverseCommand(0.5));
    new Trigger(inputs::getRightBumper).whileTrue(shooter.shoot(0.3, 0.05));
  }

  /** Call once per loop from {@link Robot#robotPeriodic()} to sample driver input while recording. */
  public void recordPeriodic() {
    recorder.periodic();
  }

  /** Starts recording driver input, discarding whatever was previously recorded. Call from teleopInit(). */
  public void startRecording() {
    recorder.start();
  }

  /** Stops recording (if active) and saves it to the deploy directory. Call from disabledInit(). */
  public void stopAndSaveRecording() {
    if (!recorder.isRecording()) {
      return;
    }
    recorder.stop();
    InputRecording.save(recorder.getSamples(), RECORDING_FILE);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. Replays the most
   * recently saved recording (see {@link #startRecording()}/{@link #stopAndSaveRecording()}) by
   * redirecting `inputs` to a playback source for the duration of the recording; the same default
   * commands/triggers used in teleop then drive the robot exactly as it was driven when recorded.
   *
   * @return the command to run in autonomous, or null if no recording has been saved yet
   */
  public Command getAutonomousCommand() {
    List<DriverInputSample> samples = InputRecording.load(RECORDING_FILE);
    if (samples.isEmpty()) {
      return null;
    }

    return new ReplayInputsCommand(inputs, liveInputs, samples);
  }

}