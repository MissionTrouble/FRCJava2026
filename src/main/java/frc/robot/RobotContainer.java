// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.Joysticks;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverController =
      new CommandXboxController(Joysticks.DRIVER_CONTROLLER_PORT);

  // The robot's subsystems and commands are defined here...
    private final Intake intake;
    private final HopperSubsystem hopper;
    private final ShooterSubsystem shooter;
    private final Vision vision;
    private final Swerve swerve;
    private final Odometry odometry;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    odometry = new Odometry();

    this.vision = new Vision();

    swerve = new Swerve(odometry,
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      () -> driverController.getRawAxis(4),
      () -> driverController.getRightTriggerAxis()
    );

    if (Robot.isReal()) {
      intake = new Intake();
      hopper = new HopperSubsystem();
      shooter = new ShooterSubsystem(odometry);
      intake.pivot.setDefaultCommand(intake.stopPivot());
      intake.roller.setDefaultCommand(intake.stopRoller());
      hopper.setDefaultCommand(hopper.stopCommand());
    } else {
      intake = null;
      hopper = null;
      shooter = null;
    }

    swerve.swerve.setDefaultCommand(swerve.joystickDriveCommand());
    vision.setDefaultCommand(vision.updatePoseCommand(odometry));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link CommandXboxController}.
   */
  private void configureBindings() {
    driverController.x().toggleOnTrue(swerve.pointToHubCommand());
    driverController.start().onTrue(odometry.resetGyro());
    driverController.back().whileTrue(odometry.resetOdometry(swerve.swerve, vision));
    if (Robot.isReal()) {
      driverController.povDown().whileTrue(intake.reverseRoller(0.5));
      driverController.y().toggleOnTrue(intake.startRoller(0.5));
      driverController.leftBumper().onTrue(intake.togglePivot(0.5, 0.1));
      driverController.leftBumper().toggleOnTrue(hopper.startCommand(0.5));
      driverController.povUp().whileTrue(hopper.reverseCommand(0.5));
      driverController.rightBumper().whileTrue(shooter.shoot(0.3, 0.05));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

}