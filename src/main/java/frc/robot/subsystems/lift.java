// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.PIDSlot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class lift extends SubsystemBase {
  /** Creates a new lift. */
  private ThriftyNova m_lift1;
  private ThriftyNova m_lift2;
  private final ThriftyNova m_intake;
  private final SparkMax m_arm;
  private final SparkMax m_wrist;
  
  private final RelativeEncoder m_armEncoder;
  private final RelativeEncoder m_wristEncoder;
  
  private final SparkClosedLoopController m_armController;
  private final SparkClosedLoopController m_wristController;
  
  private SparkMaxConfig armConfig;
  private SparkMaxConfig wristConfig;
  
  public lift() { 

  m_lift1 = new ThriftyNova(0);
  m_lift1.setBrakeMode(true); // brake mode
  m_lift1.setInverted(false); // not inverted 
  m_lift1.setRampUp(0.25);    // 1/4 second ramp up
  m_lift1.setRampDown(0.25);  // tiny ramp dowm
  m_lift1.setMaxOutput(.25, 0.5);  // full power for forward because the 
                              // system is fighting against gravity
                              // limits power on reverse because the 
                              // system is falling with gravity
                              
  m_lift1.setSoftLimits(0, 2000); // constrain the motor [0, 4pi]
  m_lift1.enableSoftLimits(true);       // enable the soft limits   
  m_lift1.setMaxCurrent(CurrentType.SUPPLY, 50); // set a 50amp current limit
                                             // on supply side
      
  m_lift1.useEncoderType(EncoderType.INTERNAL); // use internal NEO encoder
  m_lift1.usePIDSlot(PIDSlot.SLOT0);           // use the first PID slot
      
    // Configure the first PID slot
  m_lift1.pid0.setP(0.008); 
  m_lift1.pid0.setI(0);
  m_lift1.pid0.setD(0.1);
  m_lift1.pid0.setFF(0);

  m_lift2 = new ThriftyNova(1);
  m_lift2.setBrakeMode(false);
  m_lift2.follow(0);
  m_lift2.setInverted(true);
  
  m_intake = new ThriftyNova(4);
    m_intake.setBrakeMode(true); // brake mode
    m_intake.setInverted(false); // not inverted 
    m_intake.setRampUp(0.25);    // 1/4 second ramp up
    m_intake.setRampDown(0.25);  // tiny ramp dowm
    m_intake.setMaxOutput(1, 1);  // full power for forward because the 
                              // system is fighting against gravity
                              // limits power on reverse because the 
                              // system is falling with gravity
                              
    //m_intake.setSoftLimits(0, 2000); // constrain the motor [0, 4pi]
    //m_intake.enableSoftLimits(true);       // enable the soft limits   
    m_intake.setMaxCurrent(CurrentType.SUPPLY, 50); // set a 50amp current limit
                                             // on supply side
      
    m_intake.useEncoderType(EncoderType.INTERNAL); // use internal NEO encoder
    m_intake.usePIDSlot(PIDSlot.SLOT0);           // use the first PID slot
      
    // Configure the first PID slot
    m_intake.pid0.setP(0.1); 
    m_intake.pid0.setD(0);

  // Iterate through errors and check them
  for (var err : m_lift1.getErrors()) {
    // The user can handle the errors
      System.err.println(err.toString());
    }
    // Clear errors here
    m_lift1.clearErrors();
    m_intake.clearErrors();

    m_arm = new SparkMax(2, MotorType.kBrushless);
    m_wrist = new SparkMax(3, MotorType.kBrushless);

    m_armController = m_arm.getClosedLoopController(); 
    m_wristController = m_wrist.getClosedLoopController();

    m_armEncoder = m_arm.getEncoder();
    m_wristEncoder = m_wrist.getEncoder();

    armConfig = new SparkMaxConfig();
    wristConfig = new SparkMaxConfig();

    armConfig.encoder.positionConversionFactor(1); 
    wristConfig.encoder.positionConversionFactor(1);  

    armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.15)
        .i(0)
        .d(0)
        .outputRange(-0.35, 0.35);
    wristConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.04)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
  
    m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void LiftLow() {
    m_lift1.setPosition(500);    
  }
  
  public void LiftMid() {
    m_lift1.setPosition(750);
  }

  public void ResetLift() {
    m_lift1.setPosition(0);
  }

  public double liftheight() {
    return m_lift1.getPosition();
  }

  public void ResetIntake() {
    m_intake.setEncoderPosition(0);
  }

  public void Intake() {
    m_intake.set(.25);
  }

  public void stopIntake() {
    m_intake.set(0);
  }

  public void ArmAng(double angle) {
    m_armController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void Wrist(double angle) {
    m_wristController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void Liftheight(double pos) {
    m_lift1.setPosition(pos);
  }

  public void pickup() {
    Liftheight(350);
    ArmAng(25);
    Wrist(-43);
  }
  public void placehigh() {
    Liftheight(1500);
    ArmAng(-20);
    Wrist(-43);
  }

  public void ready() {
    Liftheight(700);
    ArmAng(30);
    Wrist(-70);
  }
  public void ResetArm() {
    m_armEncoder.setPosition(0);
  }

  public void ResetWrist() {
    m_wristEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("height", m_lift1.getPosition());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm angle", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Wrist angle", m_wristEncoder.getPosition());
  
    }

}

