// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.procam;
import frc.robot.LimelightHelpers;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private procam m_Vision;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem m_Swerve;

  //private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  
  /** Creates a new Align. */
  public Align(DriveSubsystem swerve, procam limelight, boolean isRightScore) {
    // Use addRequirements() here to declare subsystem dependencies.
    xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0, 0);  // Vertical movement
    yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0, 0);  // Horitontal movement
    rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.m_Vision = limelight;
    this.m_Swerve = swerve;
    addRequirements(swerve, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT : -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Vision.getTV()) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);
      
      double xSpeed = xController.calculate(postions[2]);
      //SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);
      //m_Swerve.drive(alignRequest.withVelocityX(yController.getError() < VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0).withRotationalRate(rotValue).withVelocityY(ySpeed));

      //m_Swerve.setControl(alignRequest.withVelocityX(yController.getError() < VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT ? 0 : 0).withRotationalRate(rotValue).withVelocityY(ySpeed));
      //m_Swerve.drive();

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ){//||
          //!xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_Swerve.drive(0,0,0,true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(VisionConstants.waitTime) ||
        stopTimer.hasElapsed(VisionConstants.validationTime);
  }
}
