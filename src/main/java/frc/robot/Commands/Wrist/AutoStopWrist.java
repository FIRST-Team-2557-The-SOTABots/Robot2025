// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.FourBar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Outake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStopWrist extends SequentialCommandGroup {
  /** Creates a new AutoStopWrist. */
  public AutoStopWrist(Lift m_lift, Wrist m_wrist, Outake m_outake , FourBar m_fourbar, Intake m_intake) {
    //get everything in the right position especly so the lift doesnt hit the outake
    addCommands(new RunCommand(
        () -> {m_fourbar.setPostion(Constants.FourBarConstants.FourBarPostion.kPositionResting);
          m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionSafe);
          m_lift.setPostion(Constants.LiftConstants.LiftHeight.kPositionResting);},
        m_fourbar, m_wrist, m_lift).until(m_lift::isInPostion),
      new RunCommand(
    //make sure we dont shoot the coral into the climber
        () -> m_wrist.setPosition(Constants.WristConstants.WristPostion.kPositionResting),
          m_wrist).until(m_fourbar::isResting),
    //move the coral until it is in the outake
      new RunCommand(
        () -> {m_intake.setVoltage(Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery,
          Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery);
          m_outake.setVoltage(Constants.OutakeConstants.OutakeSpeeds.kSpeedDelvery);},
        m_intake, m_outake).until(m_outake::hasCoral),
      //move the coral a little bit more
      new RunCommand(
        () -> {m_intake.setVoltage(Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery,
          Constants.IntakeConstants.IntakeSpeeds.kSpeedDelvery);
          m_outake.setVoltage(Constants.OutakeConstants.OutakeSpeeds.kSpeedDelvery);},
        m_intake, m_outake).withTimeout(.2),
    //set everything too stop
      new RunCommand(
        () -> {m_intake.setVoltage(0, 0);
          m_outake.setVoltage(0);},
          m_intake, m_outake));
        addRequirements(m_fourbar, m_wrist, m_lift, m_outake, m_intake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
