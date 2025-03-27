// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.nio.file.attribute.PosixFileAttributes;

import com.revrobotics.RelativeEncoder;


public class winch extends SubsystemBase {
  /** Creates a new winch. */
  private SparkMax m_winch;
  private final RelativeEncoder m_winchEnc;
  private final SparkClosedLoopController m_winchController;
  private SparkMaxConfig winchConfig;

  public winch() {
    m_winch = new SparkMax(5, MotorType.kBrushless);
    m_winchController = m_winch.getClosedLoopController();
    m_winchEnc = m_winch.getEncoder();
    winchConfig = new SparkMaxConfig();
    winchConfig.encoder.positionConversionFactor(1);

    winchConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(.1)
        .outputRange(-0.5, 0.5);
        
    m_winch.configure(winchConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveClimber(double pos) {
    m_winchController.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public void extendClimb() {
    moveClimber(-10);
  }

  public void retractClimb() {
    moveClimber(80);
  }

  public void winchout() {
    m_winch.set(0.25);
  }

  public void winchin() {
    m_winch.set(-0.25);
  }

  public double getWinch() {
    return m_winchEnc.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch Height", getWinch());
  }
}
