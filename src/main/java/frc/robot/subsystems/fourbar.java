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

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class FourBar extends SubsystemBase {
  private SparkMax m_motor;
  private RelativeEncoder m_motorEncoder;
  private SparkClosedLoopController m_motorPID;
  private double position = 0;
  /** Creates a new fourbar. */
  public FourBar() {
    m_motor = new SparkMax(Constants.FourBarConstants.kRightCANid, Constants.FourBarConstants.kRightMotorType);
    m_motor.configure(Configs.FourBar.rightConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getClosedLoopController();
  }

  public void setPostion(double position) {
    this.position = position;
  }

  @Override
  public void periodic() {
    m_motorPID.setReference(position, ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
