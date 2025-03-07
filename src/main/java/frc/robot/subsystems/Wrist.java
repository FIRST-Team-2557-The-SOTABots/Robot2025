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

public class Wrist extends SubsystemBase {
  private SparkMax m_motor;
  private RelativeEncoder m_motorEncoder;
  private SparkClosedLoopController m_motorPID;
  private double position;
  private Lift m_lift;

  /** Creates a new Wrist. */
  public Wrist(Lift m_lift) {
    this.m_lift = m_lift;
    m_motor = new SparkMax(Constants.WristConstants.kMotorCANid, Constants.WristConstants.kMotorType);
    m_motor.configure(Configs.Wrist.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_motorEncoder = m_motor.getEncoder();
    m_motorPID = m_motor.getClosedLoopController();
  }

  public void setZero() {
    while (m_motor.getBusVoltage() < Constants.WristConstants.kZeroTolerance) {
      position = m_motorEncoder.getPosition() + Constants.WristConstants.kZeroSpeed;
    }
    m_motorEncoder.setPosition(0);
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void setPosition(double position) {
    if (m_lift.isInPostion())
      this.position = position;
  }


  @Override
  public void periodic() {
    m_motorPID.setReference(position, ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
