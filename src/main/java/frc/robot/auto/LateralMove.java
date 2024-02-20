// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveSubsystem;

public class LateralMove extends CommandBase {
  /** Creates a new man idk. */
  private final DriveSubsystem driveSubsystem;
  public static Pose2d currentPosition;
  public static double currentPositionX;
  public static double currentPositionY;

  public static Pose2d originalPos;
  public static double originalPosX;
  public static double originalPosY;

  public LateralMove(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    currentPosition = driveSubsystem.swerveDrivePoseEstimator.getEstimatedPosition();
  }

  double dP = 0.0;
  double dI = 0.0;
  double dD = 0.0;
  PIDController lateralPID = new PIDController(dP, dI, dD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPosition = driveSubsystem.swerveDrivePoseEstimator.getEstimatedPosition();
    originalPos = currentPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = driveSubsystem.swerveDrivePoseEstimator.getEstimatedPosition(); //GET THE ESTIMATED POSITION FROM THE POSE ESTIMATOR
    currentPositionX = driveSubsystem.swerveDrivePoseEstimator.getEstimatedPosition().getX(); //GET THE X COMPONENT OF THAT POSITION
    SmartDashboard.putNumber("currentPosX", currentPositionX); //PUT THE CURRENT X POSITION ON SMARTDASHBOARD
    SmartDashboard.putNumber("currentPosX", originalPosX); //PUT THE CURRENT X POSITION ON SMARTDASHBOARD
    double moveValue = lateralPID.calculate(currentPositionX, (originalPosX) + 5); //CALCULATE PID CONSTANTS TO MAKE THE CURRENT POSITION = ORIGINAL POS X
    SmartDashboard.putNumber("TurnValue", moveValue);
    driveSubsystem.drive(moveValue, moveValue, 0, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setWheelsX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


