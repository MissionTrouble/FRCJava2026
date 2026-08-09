// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Boolean VISION_SIM = true;
  public static final Translation2d HUB_LOCATION = new Translation2d(4.62534, 4.035);
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  

  public static double clamp(double value, double min, double max) {
        // This unusual condition allows keeping only one branch
        // on common path when min < max and neither of them is NaN.
        // If min == max, we should additionally check for +0.0/-0.0 case,
        // so we're still visiting the if statement.
        if (!(min < max)) { // min greater than, equal to, or unordered with respect to max; NaN values are unordered
            if (Double.isNaN(min)) {
                throw new IllegalArgumentException("min is NaN");
            }
            if (Double.isNaN(max)) {
                throw new IllegalArgumentException("max is NaN");
            }
            if (Double.compare(min, max) > 0) {
                throw new IllegalArgumentException(min + " > " + max);
            }
            // Fall-through if min and max are exactly equal (or min = -0.0 and max = +0.0)
            // and none of them is NaN
        }
        return Math.min(max, Math.max(value, min));
    }


  public static DCMotor getNeoSimMotor() {
      double NEO_FREE_RPM = 5676.0;
      double NEO_NOMINAL_VOLTAGE = 12.0;
      double NEO_STALL_TORQUE = 2.6;
      double NEO_STALL_CURRENT = 105.0;
      double NEO_FREE_CURRENT = 1.8;
      DCMotor neo = new DCMotor(NEO_NOMINAL_VOLTAGE, NEO_STALL_TORQUE, NEO_STALL_CURRENT, NEO_FREE_CURRENT, NEO_FREE_RPM, 1);
      return neo;
  }

 public static final class DriveConstants {
    private static final double ROBOT_WIDTH = Units.inchesToMeters(26);// Distance between right and left wheels
    private static final double ROBOT_LENGTH = Units.inchesToMeters(21);// Distance between front and back wheels

    private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(ROBOT_LENGTH / 2,  ROBOT_WIDTH / 2);
    private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(ROBOT_LENGTH / 2,  -ROBOT_WIDTH / 2);
    private static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-ROBOT_LENGTH / 2,  ROBOT_WIDTH / 2);
    private static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-ROBOT_LENGTH / 2,  -ROBOT_WIDTH / 2);
    public static final SwerveModulePosition[] EMPTY_SWERVE_MODULE_POSITIONS = {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
    };
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT_LOCATION,
          FRONT_RIGHT_LOCATION, 
          BACK_LEFT_LOCATION, 
          BACK_RIGHT_LOCATION);

    public static final double MAX_SPEED_MPS = 5;
    public static final double MAX_ANGULAR_SPEED_MPS = 4 * Math.PI;
    public static final boolean TRIGGER_BOOST = false;
    public static final double MAX_ACCELERATION = 5;
    public static final double MAX_ANGULAR_ACCELERATION = 10;
    public static final Boolean FIELD_ORIENTED = true;

 }

  public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75; //mk4 L2
        public static final double DRIVE_ENCODER_ROT_2_METERS = (Math.PI * WHEEL_DIAMETER_METERS)/DRIVE_MOTOR_GEAR_RATIO;
        public static final double MODULE_TURNING_CONTROLER_P = 0.9;
        public static final double MODULE_TURNING_CONTROLER_I = 0;
        public static final double MODULE_TURNING_CONTROLER_D = 0.1;
    }
  public static class Joysticks {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
  public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0,0,0,new Rotation3d());
  public static SparkBaseConfig getDefaultMotorConfig() {
      SparkBaseConfig config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake);
      return config;
  }
  public static SparkBaseConfig getSwerveRotationMotorConfig() {
      SparkBaseConfig config = getDefaultMotorConfig();
      config.encoder
      .positionConversionFactor(ModuleConstants.DRIVE_ENCODER_ROT_2_METERS);

      return config;
  }
  public static SparkBaseConfig getSwerveDriveMotorConfig() {
      SparkBaseConfig config = getDefaultMotorConfig();
      config.analogSensor.positionConversionFactor(2*Math.PI/3.3);
      config.closedLoop
            .feedbackSensor(FeedbackSensor.kAnalogSensor)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI,Math.PI)
            .pid(ModuleConstants.MODULE_TURNING_CONTROLER_P, 
              ModuleConstants.MODULE_TURNING_CONTROLER_I, 
              ModuleConstants.MODULE_TURNING_CONTROLER_D);
      return config;
  }
  public static class MotorInfo {
      public final int CAN_ID;
      public final boolean REVERSED;

      public MotorInfo(int id, boolean reversed) {
          this.CAN_ID = id;
          this.REVERSED = reversed;
      }
  }

  public static final class EncoderOffsets {
        public static final double FRONT_LEFT = 3.99;
        public static final double FRONT_RIGHT = 4.3;
        public static final double BACK_LEFT = 5.6;
        public static final double BACK_RIGHT = 1.42;
  }

public static class MOTORS {
    public static final MotorInfo EXAMPLE = new MotorInfo(-1, false);

    public static final MotorInfo INTAKE_ROLLER = new MotorInfo(0, false);
    public static final MotorInfo INTAKE_PIVOT = new MotorInfo(0, false);
    public static final MotorInfo HOPPER = new MotorInfo(0, false);
    public static final MotorInfo FLYWHEEL = new MotorInfo(0, false);
    public static final MotorInfo HOOD = new MotorInfo(0, false);
    public static final MotorInfo CHUTE = new MotorInfo(0, false);


    public static final MotorInfo FRONT_LEFT_SWERVE_POWER = new MotorInfo(1, false);
    public static final MotorInfo FRONT_RIGHT_SWERVE_POWER = new MotorInfo(3, false);
    public static final MotorInfo BACK_LEFT_SWERVE_POWER = new MotorInfo(5, false);
    public static final MotorInfo BACK_RIGHT_SWERVE_POWER = new MotorInfo(7, false);

    public static final MotorInfo FRONT_LEFT_SWERVE_ROTATION = new MotorInfo(2, false);
    public static final MotorInfo FRONT_RIGHT_SWERVE_ROTATION = new MotorInfo(4, false);
    public static final MotorInfo BACK_LEFT_SWERVE_ROTATION = new MotorInfo(6, false);
    public static final MotorInfo BACK_RIGHT_SWERVE_ROTATION = new MotorInfo(8, false);
}
}
