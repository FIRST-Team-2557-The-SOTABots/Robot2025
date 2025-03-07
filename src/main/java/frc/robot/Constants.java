// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class LiftConstants {
    public static final int kRightCANid = 12;
    public static final int kLeftCANid = 13;

    public static final MotorType kRightMotorType = MotorType.kBrushless;
    public static final MotorType kLeftMotorType = MotorType.kBrushless;

    public static final boolean kRightInverted = false;
    public static final boolean kLeftInverted = true;

    public static final int kRightCurrentLimit = 70;
    public static final int kLeftCurrentLimit = 70;

    public static final double kLiftHighP = 1.5;
    public static final double kLiftLowP = .5;
    public static final double kLiftI = .5;
    public static final double kLiftD = 0.0;
    public static final double kLiftMaxSpeed = 15;
    public static final double kLiftMaxAcceleration = 5;

    public static final double kLiftGearRatio = 1;

    public final class LiftHeight {
      public static final double kPositionResting = -.1;
      // public static final double kPositionSafe = 5;
      public static final double kPositionL2 = 39;
      public static final double kPositionL3 = 65;
      public static final double kPositionL4 = 100;
      public static final double kPositionCoralStation = 7;
    }

    public static final double kZeroSpeed = 0.01;
    public static final double kZeroTolerance = 10;

  }

  public static final class WristConstants {
    public static final int kMotorCANid = 14;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kInverted = true;

    public static final int kCurrentLimit = 80;

    public static final double kWristP = .2;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public final class WristPostion {
      public static final double kPositionResting = 4;
      public static final double kPositionSafe = 15;
      public static final double kPositionL23 = 35;
      public static final double kPositionL4 = 30;
      public static final double kPosistionCoralStation = 15;
    }

    public static final double kZeroSpeed = 0.2;
    public static final double kZeroTolerance = 10;

  }

  public static final class FourBarConstants {
    public static final int kMotorCANid = 9;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kInverted = true;

    public static final int kCurrentLimit = 50;

    public static final double kFourBarP = 1.5;
    public static final double kFourBarI = 0.0;
    public static final double kFourBarD = 0.0;

    public final class FourBarPostion {
      public static final double kPositionResting = 0.31;
      public static final double kPositionCoral = 0.59;
      public static final double kPositionAlgae = 0.62;
      public static final double kPositionClimb = 0.45;
    }

  }

  public static final class OutakeConstants {
    public static final int kMotorCANid = 15;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kInverted = true;

    public static final int kCurrentLimit = 50;

    public static final double kOutakeGearRatio = 1;

    public static final double kOutakeP = .01;
    public static final double kOutakeI = 0.0;
    public static final double kOutakeD = 0.0;

    public static final double kOutakeVolts = 7;

  }

  public static final class IntakeConstants {
    public static final int kIntakeCANid = 10;
    public static final int kDeliveryCANid = 11;

    public static final MotorType kIntakeMotorType = MotorType.kBrushless;
    public static final MotorType kDeliveryMotorType = MotorType.kBrushless;

    public static final boolean kIntakeInverted = true;
    public static final boolean kDeliveryInverted = true;

    public static final int kIntakeCurrentLimit = 50;
    public static final int kDeliveryCurrentLimit = 50;

    public static final double kIntakeVolts = 10;
    public static final double kDeliveryVolts = 11;

    // public static final double kIntakeP = .001;
    // public static final double kIntakeI = 0.0;
    // public static final double kIntakeD = 0.0;

    // public final class IntakeSpeeds {
    //   public static final double kSpeedCoralRight = 10;
    //   public static final double kSpeedCoralLeft = 10;
    //   public static final double kSpeedAlgeaRight = 10;
    //   public static final double kSpeedAlgeaLeft = 10;
    //   public static final double kSpeedDelvery = 10;
    // }
  }

  public static final class ClimberConstants {
    public static final int kMotorCANid = 16;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final boolean kMotorInverted = false;

    public static final int kMotorCurrentLimmit = 50;

  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
