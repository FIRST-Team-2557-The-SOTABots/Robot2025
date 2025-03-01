// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkMax m_motor;

  /** Creates a new Climber. */
  public Climber() {
    m_motor = new SparkMax(Constants.ClimberConstants.kMotorCANid,
        Constants.ClimberConstants.kMotorType);

    m_motor.configure(Configs.Climber.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setVoltage(double voltage) {
    if (voltage > 5) {
      m_motor.setVoltage(voltage);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
