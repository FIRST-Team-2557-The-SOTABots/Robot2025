// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToAprilTag extends Command {
  private DriveSubsystem mDrive;
  private PIDController mTurn;
  private PIDController mfwd;
  private PIDController mside;
  /** Creates a new AprilTag. */
  public MoveToAprilTag(DriveSubsystem drive) {
    this.mDrive = drive;
    PIDController fwd = new PIDController(.05, .0, .1);
    this.mfwd = fwd;

    PIDController side = new PIDController(.05, 0, .1);
    this.mside = side;

    PIDController turn = new PIDController(.013, 0.01, 0.01);
    this.mTurn = turn;

    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double correctedTX(){

    if (Math.abs(LimelightHelpers.getTX("")) < 2){
      return 0;
    } else {
      return LimelightHelpers.getTX("");
    }
  }

  public double correctedTY(){

    if (Math.abs(LimelightHelpers.getTY("")) < 2){
      return 0;
    } else {
      return LimelightHelpers.getTY("");
    }
  }

  public double parallelDiff(){
    return mDrive.getHeading() - (-92);
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.drive(
      //-mfwd.calculate(correctedTY(), 0),
      0,
      //mside.calculate(parallelDiff(),0),
      0,
      mTurn.calculate(correctedTX(), 0),
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
