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

public class Intake extends SubsystemBase {
  private SparkMax m_right;
  private SparkMax m_left;

  private DigitalInput proxSensor;
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

    proxSensor = new DigitalInput(8);
  }

  public boolean hasCoral(){
    return !proxSensor.get();
  }

  public void setVoltage(double rightSpeed, double leftSpeed) {
    m_left.setVoltage(leftSpeed);
    m_right.setVoltage(rightSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake has Coral", hasCoral());
  }
}
