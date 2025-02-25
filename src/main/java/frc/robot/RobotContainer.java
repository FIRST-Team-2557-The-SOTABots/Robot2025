// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FourBar;
import frc.robot.subsystems.Outake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.apriltag.AprilTag;
import frc.robot.Commands.Drive.MoveToAprilTag;
import frc.robot.Commands.Intake.AutoStopIntake;
import frc.robot.Commands.Lift.LiftAndWristMove;
import frc.robot.Commands.Wrist.AutoStopWrist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import com.pathplanner.lib.auto.AutoBuilder;


import java.util.List;

//import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final FourBar m_fourBar = new FourBar();
  private final Lift m_lift = new Lift();
  private final Wrist m_wrist = new Wrist(m_lift);
  private final Outake m_outake = new Outake();
  private final Climber m_climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  //private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();;
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_fourBar.setDefaultCommand(new RunCommand(
        () -> m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting),
        m_fourBar));
    
    m_lift.setDefaultCommand(new RunCommand(
        () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting),
        m_lift));

    m_wrist.setDefaultCommand(new RunCommand(
        () -> m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting),
         m_wrist));

    m_climber.setDefaultCommand(new RunCommand(
        () -> m_climber.setVoltage((-m_manipulatorController.getLeftY()) * 12),
         m_climber));

    // m_wrist.setDefaultCommand(new RunCommand(
    //     () -> m_wrist.setSpeed(-m_manipulatorController.getLeftY()),
    //     m_wrist));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    // ...

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(Commands.runOnce(
            () -> m_robotDrive.
            zeroHeading(),
            m_robotDrive));

    // new JoystickButton(m_manipulatorController, Button.kStart.value)
    //     .whileTrue(new RunCommand(
    //         () -> {m_wrist.setZero();
    //             m_lift.setZero();},
    //          m_wrist, m_lift));

    // new JoystickButton(m_manipulatorController, Button.kA.value)
    //     .onTrue(new AutoStopIntake(m_fourBar, m_intake)
    //         .andThen(new AutoStopWrist(m_lift, m_wrist, m_outake, m_fourBar, m_intake)))
    //     .onFalse(Commands.runOnce(
    //         () -> m_intake.setVoltage(0, 0), 
    //         m_intake));

    new JoystickButton(m_manipulatorController, Button.kA.value)
        .onTrue(new AutoStopIntake(m_fourBar, m_intake))
        .onFalse(Commands.runOnce(
            () -> {m_intake.setVoltage(0, 0);
            m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);}, 
            m_intake, m_fourBar));

    new JoystickButton(m_manipulatorController, Button.kLeftStick.value)
        .onTrue(new RunCommand(
            () -> {m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionAlgae);
                m_intake.setVoltage(Constants.IntakeConstants.IntakeSpeeds.kSpeedAlgeaRight,
                Constants.IntakeConstants.IntakeSpeeds.kSpeedAlgeaLeft);},
                m_fourBar, m_intake))
        .onFalse(Commands.runOnce(
            () -> m_intake.setVoltage(0, 0), 
            m_intake));

        new JoystickButton(m_manipulatorController, Button.kLeftBumper.value)
            .onTrue(new RunCommand(
                () -> {m_intake.setVoltage(Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery,
                    Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery);
                    m_outake.setVoltage(Constants.OutakeConstants.OutakeSpeeds.kSpeedDelvery);}
                        ,m_intake, m_outake))
            .onFalse(new RunCommand(
                () -> {m_intake.setVoltage(0, 0);
                    m_outake.setVoltage(0);}
                        ,m_intake, m_outake));

        new JoystickButton(m_manipulatorController, Button.kRightBumper.value)
            .onTrue(new RunCommand(
                () -> {m_intake.setVoltage(-Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery, 
                    -Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery);
                    m_outake.setVoltage(-Constants.OutakeConstants.OutakeSpeeds.kSpeedDelvery);}
                        ,m_intake, m_outake))
            .onFalse(new RunCommand(
                () -> {m_intake.setVoltage(0, 0);
                    m_outake.setVoltage(0);}
                        ,m_intake, m_outake));

            //     new JoystickButton(m_manipulatorController, Button.kY.value)
            // .onTrue(new RunCommand(
            //     () -> {m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionL2);
            //     m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL2);},
            //     m_wrist,m_lift))
            // .onFalse( new RunCommand(
            //     () -> {m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting);
            //     m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting);},
            //     m_wrist,m_lift));

        // new JoystickButton(m_manipulatorController, Button.kB.value)
        //     .onTrue(Commands.runOnce(
        //         () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL4),
        //         m_lift))
        //     .onFalse( new RunCommand(
        //         () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting),
        //         m_lift));

        // new JoystickButton(m_manipulatorController, Button.kX.value)
        //     .onTrue(new RunCommand(
        //         () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL3),
        //         m_lift))
        //     .onFalse( new RunCommand(
        //         () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting),
        //         m_lift));

        new JoystickButton(m_manipulatorController, Button.kB.value)
        .onTrue(new LiftAndWristMove(m_lift, m_wrist, 
            Constants.LiftConstants.LiftHeight.kPositionCoralStation,
            Constants.WristConstants.WristPostion.kPositionResting))
        .onFalse(new LiftAndWristMove(m_lift, m_wrist, 
            Constants.LiftConstants.LiftHeight.kPositionResting,
            Constants.WristConstants.WristPostion.kPositionResting));

        new JoystickButton(m_manipulatorController, Button.kY.value)
            .onTrue(new LiftAndWristMove(m_lift, m_wrist, 
                Constants.LiftConstants.LiftHeight.kPositionL3,
                Constants.WristConstants.WristPostion.kPositionL23))
            .onFalse(new LiftAndWristMove(m_lift, m_wrist, 
                Constants.LiftConstants.LiftHeight.kPositionResting,
                Constants.WristConstants.WristPostion.kPositionResting));

        new JoystickButton(m_manipulatorController, Button.kX.value)
            .onTrue(new LiftAndWristMove(m_lift, m_wrist, 
                Constants.LiftConstants.LiftHeight.kPositionL2,
                Constants.WristConstants.WristPostion.kPositionL23))
            .onFalse(new LiftAndWristMove(m_lift, m_wrist, 
                Constants.LiftConstants.LiftHeight.kPositionResting,
                Constants.WristConstants.WristPostion.kPositionResting));

        // new JoystickButton(m_manipulatorController, Button.kY.value)
        //     .onTrue(new RunCommand(
        //         () -> m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionSafe),
        //         m_wrist))
        //     .onFalse( new RunCommand(
        //         () -> m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting),
        //         m_wrist));

        // new JoystickButton(m_manipulatorController, Button.kRightStick.value)
        //         .whileTrue(new RunCommand(
        //             () -> {m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionClimb);
        //             m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionL23);}, 
        //             m_fourBar,m_wrist))
        //         .onFalse(new RunCommand(
        //             () -> {m_fourBar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);
        //                 m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting);}, 
        //                 m_fourBar,m_wrist));

//     new JoystickButton(m_manipulatorController, Button.kY.value)
//         .onTrue(new RunCommand(
//             () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL3), 
//             m_lift));

//     new JoystickButton(m_manipulatorController, Button.kX.value)
//         .onTrue(new RunCommand(
//             () -> m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionL2), 
//             m_lift));

    new JoystickButton(m_manipulatorController, Button.kStart.value)
        .onTrue(Commands.runOnce(
            () -> m_lift.resetLift(),
                m_lift));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
