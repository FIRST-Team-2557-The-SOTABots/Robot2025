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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Outake extends SubsystemBase {
  private SparkMax m_motor;
  private double speed = 0;
  private DigitalInput m_limitswitch;
  /** Creates a new Outake. */
  public Outake() {
    m_motor = new SparkMax(Constants.OutakeConstants.kMotorCANid, Constants.OutakeConstants.kMotorType);
    m_motor.configure(Configs.Outake.motorConfig, 
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_limitswitch = new DigitalInput(9);
  }

  public void setVoltage(double speed) {
    m_motor.setVoltage(speed);
  }

  public boolean hasCoral(){
    return m_limitswitch.get();
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("outake has coral", hasCoral());
    // This method will be called once per scheduler run
  }
}
