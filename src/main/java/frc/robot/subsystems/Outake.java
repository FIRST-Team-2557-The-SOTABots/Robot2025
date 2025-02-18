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

public class Outake extends SubsystemBase {
  private SparkMax m_motor;
  private SparkClosedLoopController m_motorPID;
  private RelativeEncoder m_motorEncoder;
  private double speed = 0;
  /** Creates a new Outake. */
  public Outake() {
    m_motor = new SparkMax(Constants.OutakeConstants.kMotorCANid, Constants.OutakeConstants.kMotorType);
    m_motor.configure(Configs.Outake.motorConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getClosedLoopController();
  }

  public void setPostion(double speed) {
    this.speed = speed;
  }

  

  @Override
  public void periodic() {
    m_motorPID.setReference(speed, ControlType.kVelocity);
    // This method will be called once per scheduler run
  }
}
