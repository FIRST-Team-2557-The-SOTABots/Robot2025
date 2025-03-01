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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  private SparkMax m_right;
  private RelativeEncoder m_rightEncoder;
  private PIDController m_PID;
  private double position;

  private SparkMax m_left;
  private RelativeEncoder m_leftEncoder;

  /** Creates a new Lift. */
  public Lift() {
    m_right = new SparkMax(Constants.LiftConstants.kRightCANid, Constants.LiftConstants.kRightMotorType);
    m_right.configure(Configs.Lift.rightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_left = new SparkMax(Constants.LiftConstants.kLeftCANid, Constants.LiftConstants.kLeftMotorType);
    m_left.configure(Configs.Lift.leftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_PID = new PIDController(Constants.LiftConstants.kLiftLowP,
        Constants.LiftConstants.kLiftI,
        Constants.LiftConstants.kLiftD);
  }

  public void setVoltage(double voltage) {
    m_left.set(voltage);
    m_right.set(voltage);
  }

  public boolean isInPostion() {
    return (m_right.getEncoder().getPosition() < 3);
  }

  public void setPostion(double position) {
    if (position < m_right.getEncoder().getPosition()) {
      m_PID.setP(Constants.LiftConstants.kLiftLowP);
      m_PID.setI(0);
    } else {
      m_PID.setP(Constants.LiftConstants.kLiftHighP);
      m_PID.setI(Constants.LiftConstants.kLiftI);
    }
    this.position = position;
  }

  public void setZero() {
    // while(m_right.getBusVoltage() < Constants.LiftConstants.kZeroTolerance) {
    // position = m_rightEncoder.getPosition() -
    // Constants.WristConstants.kZeroSpeed;
    // }
    // m_rightEncoder.setPosition(0);
    // m_leftEncoder.setPosition(0);
  }

  public double getOutput() {
    if ((position < m_right.getEncoder().getPosition() + 1) && (position > m_right.getEncoder().getPosition() - 1)) {
      return .1;
    } else {
      return m_PID.calculate(m_right.getEncoder().getPosition(), position);
    }
  }

  public void resetLift() {
    m_right.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    m_right.set(MathUtil.clamp(getOutput(), -.3, .3));
    m_left.set(MathUtil.clamp(getOutput(), -.3, .3));
    SmartDashboard.putNumber("lift PID", m_PID.calculate(m_right.getEncoder().getPosition(), position));
    SmartDashboard.putNumber("lift setpoint", position);
    SmartDashboard.putNumber("lift postion", m_right.getEncoder().getPosition());
    // SmartDashboard.putNumber("lift MP", m_right.GET());
    SmartDashboard.putNumber("lift P", m_PID.getP());
    SmartDashboard.putNumber("period", m_PID.getPeriod());

    SmartDashboard.putNumber("lift error", m_PID.getError());
    SmartDashboard.putNumber("lift error acc", m_PID.getAccumulatedError());
    // This method will be called once per scheduler run
  }
}
