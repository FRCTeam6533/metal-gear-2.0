// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final SparkMax m_lift1;
  private final SparkMax m_lift2;
  private final ThriftyNova m_intake;
  private final SparkMax m_arm;
  private final SparkMax m_wrist;
  
  private final RelativeEncoder m_lift1Encoder;
  private final RelativeEncoder m_lift2Encoder;
  private final RelativeEncoder m_armEncoder;
  private final RelativeEncoder m_wristEncoder;
  
  private final SparkClosedLoopController m_lift1Controller;
  private final SparkClosedLoopController m_lift2Controller;
  private final SparkClosedLoopController m_armController;
  private final SparkClosedLoopController m_wristController;
  
  private SparkMaxConfig lift1Config;
  private SparkMaxConfig lift2Config;
  private SparkMaxConfig armConfig;
  private SparkMaxConfig wristConfig;
  
  public lift() { 
  
  m_intake = new ThriftyNova(4);
    m_intake.setBrakeMode(true); // brake mode
    m_intake.setInverted(false); // not inverted 
    m_intake.setRampUp(0.25);    // 1/4 second ramp up
    m_intake.setRampDown(0.25);  // tiny ramp dowm
    m_intake.setMaxOutput(.25, .25);  // full power for forward because the 
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
    m_intake.pid0.setP(0.01); 
    m_intake.pid0.setD(0);

  // Iterate through errors and check them
  for (var err : m_intake.getErrors()) {
    // The user can handle the errors
      System.err.println(err.toString());
    }
    // Clear errors here
    m_intake.clearErrors();

    m_lift1 = new SparkMax(6, MotorType.kBrushless);
    m_lift2 = new SparkMax(7, MotorType.kBrushless);
    m_arm = new SparkMax(2, MotorType.kBrushless);
    m_wrist = new SparkMax(3, MotorType.kBrushless);

    m_lift1Controller = m_lift1.getClosedLoopController();
    m_lift2Controller = m_lift2.getClosedLoopController();
    m_armController = m_arm.getClosedLoopController(); 
    m_wristController = m_wrist.getClosedLoopController();

    m_lift1Encoder = m_lift1.getEncoder();
    m_lift2Encoder = m_lift2.getEncoder();
    m_armEncoder = m_arm.getEncoder();
    m_wristEncoder = m_wrist.getEncoder();

    lift1Config = new SparkMaxConfig();
    lift2Config = new SparkMaxConfig();
    armConfig = new SparkMaxConfig();
    wristConfig = new SparkMaxConfig();

    lift1Config.encoder.positionConversionFactor(1);
    lift2Config.encoder.positionConversionFactor(1);
    armConfig.encoder.positionConversionFactor(1); 
    wristConfig.encoder.positionConversionFactor(1);  

    lift1Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.05)
        .i(0)
        .d(.1)
        .outputRange(-0.5, 0.75);
    lift2Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.000)
        .i(0)
        .d(0)
        .outputRange(-0.5, 0.75);
    armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.01)
        .i(0)
        .d(.01)
        .outputRange(-0.35, 0.35);
    wristConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.02)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
  
    m_lift1.configure(lift1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_lift2.configure(lift2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void Resetarm() {
    m_lift1Encoder.setPosition(0);
    m_lift2Encoder.setPosition(0);
    m_armEncoder.setPosition(0);
    m_wristEncoder.setPosition(0);
  }

  public void ResetIntake() {
    m_intake.setEncoderPosition(0);
  }

  public void Intake() {
    m_intake.set(.35);
  }

  public void LetGo() {
    //m_intake.set(-.25);
    Wrist(getWrist()-10);
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
    m_lift1Controller.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);  //max 47 at 26.5 inches, 1.77x
  }

  public double getArm() {
    return m_armEncoder.getPosition();
  }

  public double getWrist() {
    return m_wristEncoder.getPosition();
  }

  public double getheight() {
    return m_lift1Encoder.getPosition();
  }

  public double getIntake() {
    return m_intake.getPosition();
  }

  public void Pick() {
    if (getheight() > 15) //15/1.77 = 8.5" high
      pickup();
      else {
        ready();
      }
  }
  
  public void pickup() {
    Liftheight(8); // min 0, max 2000
    ArmAng(10); //to swing to place position, move in neg direction
    Wrist(-30); //to swing to place position, move in neg direction, almost straight
    Intake();
  }

  public void placeL4() {
    Liftheight(47);
    if (getheight() > 17) ArmAng(-62);
    if (getheight() > 17) Wrist(-50);
    if (getArm() < 59 && getWrist() < -47) LetGo();
  }

  public void placeL3() {
    Liftheight(20);
    if (getheight() > 17) ArmAng(-52);
    if (getheight() > 17) Wrist(-18);
    if (getArm() < -30) Liftheight(0);
    if (getheight() < 5 && getArm() < -45 && getWrist() > -13) LetGo();
  }

  public void placeL2() {
    Liftheight(20);
    if (getheight() > 17) ArmAng(-75);
    if (getheight() > 17) Wrist(-14);
    if (getArm() < -50) Liftheight(0);
    if (getheight() < 5 && getArm() < -70 && getWrist() > -10) LetGo();
  }

  public void placeL1() {
    Liftheight(20);
    if (getheight() > 17) ArmAng(-75);
    if (getheight() > 17) Wrist(-7);
    if (getArm() < -50) Liftheight(0);
    if (getArm() < -50 && getheight() < 5 && getWrist() > -10) LetGo();
  }

  public void ready() {
    Liftheight(20);
    ArmAng(10);
    Wrist(-30);
  }

  public void clearLowAlgae() {
    Liftheight(0);
    ArmAng(-64);
    Wrist(-42);
    LetGo();
  }

  public void clearHighAlgae() {
    Liftheight(28);
    ArmAng(-64);
    Wrist(-42);
    LetGo();
  }

  public void ResetArm() {
    m_armEncoder.setPosition(0);
  }

  public void ResetWrist() {
    m_wristEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("height", getheight());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm angle", getArm());
    SmartDashboard.putNumber("Wrist angle", getWrist());
    //SmartDashboard.putNumber("Intake", getIntake());
    //SmartDashboard.putBoolean("LiftP", m_lift1.pid0.setP)
    }

}