// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAprilTag extends Command {
  DriveSubsystem mDrive;
  PIDController mController;
  /** Creates a new RotateToAprilTag. */
  public RotateToAprilTag(DriveSubsystem drive) {
    mDrive = drive;
    mController = new PIDController(0.1, 0, 0);
    mController.setTolerance(3);
    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.drive(0, 0, mController.calculate(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0), 0), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mController.atSetpoint();
  }
}
