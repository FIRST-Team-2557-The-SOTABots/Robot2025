// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax m_right;
  private RelativeEncoder m_rightEncoder;
  private SparkClosedLoopController m_rightPID;
  
  private SparkMax m_left;
  private RelativeEncoder m_leftEncoder;
  private SparkClosedLoopController m_leftPID;

  private double rightSpeed, leftSpeed;
  /** Creates a new intake. */
  public Intake() {
    m_right = new SparkMax(Constants.IntakeConstants.kRightCANid, Constants.IntakeConstants.kRightMotorType);
    m_right.configure(Configs.Intake.rightConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_left = new SparkMax(Constants.IntakeConstants.kLeftCANid, Constants.IntakeConstants.kLeftMotorType);
    m_left.configure(Configs.Intake.leftConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_rightEncoder = m_right.getEncoder();
    m_leftEncoder = m_left.getEncoder();

    m_rightPID = m_right.getClosedLoopController();
    m_leftPID = m_left.getClosedLoopController();
  }

  public void setSpeed(double rightSpeed, double leftSpeed) {
    this.rightSpeed = rightSpeed;
    this.leftSpeed = leftSpeed;
  }




  @Override
  public void periodic() {
    m_leftPID.setReference(leftSpeed, ControlType.kVelocity);
    m_rightPID.setReference(rightSpeed, ControlType.kVelocity);
    // This method will be called once per scheduler run
  }
}
