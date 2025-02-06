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

public class FourBar extends SubsystemBase {
  private SparkMax m_right;
  private RelativeEncoder m_rightEncoder;
  private SparkClosedLoopController m_rightPID;
  private double position = 0;
  
  private SparkMax m_left;
  /** Creates a new fourbar. */
  public FourBar() {
    m_right = new SparkMax(Constants.FourBarConstants.kRightCANid, Constants.FourBarConstants.kRightMotorType);
    m_right.configure(Configs.FourBar.rightConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_left = new SparkMax(Constants.FourBarConstants.kLeftCANid, Constants.FourBarConstants.kLeftMotorType);
    m_left.configure(Configs.FourBar.leftConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_rightEncoder = m_right.getEncoder();
    m_rightPID = m_right.getClosedLoopController();
  }

  public void setPostion(double position) {
    this.position = position;
  }

  @Override
  public void periodic() {
    m_rightPID.setReference(position, ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
