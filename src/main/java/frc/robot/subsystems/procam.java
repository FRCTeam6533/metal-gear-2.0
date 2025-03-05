// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class procam extends SubsystemBase {
  /** Creates a new procam. */


  double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
  double ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
  double ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
  boolean hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?
  
  double txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
  double tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees
  
  public procam() {

    
  
  }
public double xoff() { 
return tx; 

}
 public double yoff() {

  return ty;
 }

 public double area() {

 return ta;
 }

 public Boolean hasTarget() {

  return hasTarget;
 }

 public double xoffdeg() {

  return txnc;
 }

 public double yoffdeg() {

  return tync;
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
