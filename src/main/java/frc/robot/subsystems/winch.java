// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.PIDSlot;


public class winch extends SubsystemBase {
  /** Creates a new winch. */
  private ThriftyNova m_winch;

  public winch() {
    m_winch = new ThriftyNova(0);
    m_winch.setBrakeMode(true); // brake mode
    m_winch.setInverted(false); // not inverted 
    m_winch.setRampUp(0.25);    // 1/4 second ramp up
    m_winch.setRampDown(0.25);  // tiny ramp dowm
    m_winch.setMaxOutput(.25, 0.5);  // full power for forward because the 
                              // system is fighting against gravity
                              // limits power on reverse because the 
                              // system is falling with gravity
                              
    m_winch.setSoftLimits(0, 2000); // constrain the motor
    m_winch.enableSoftLimits(true);       // enable the soft limits   
    m_winch.setMaxCurrent(CurrentType.SUPPLY, 50); // set a 50amp current limit
                                             // on supply side
      
    m_winch.useEncoderType(EncoderType.INTERNAL); // use internal NEO encoder
    m_winch.usePIDSlot(PIDSlot.SLOT0);           // use the first PID slot
      
    // Configure the first PID slot
    m_winch.pid0.setP(0.1); 
    m_winch.pid0.setI(0);
    m_winch.pid0.setD(0.1);
    m_winch.pid0.setFF(0);
  }

  public void extendClimb() {
    m_winch.setPosition(2000);
  }

  public void retractClimb() {
    m_winch.setPosition(100);
  }

  public double getWinch() {
    return m_winch.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch Height", getWinch());
  }
}
