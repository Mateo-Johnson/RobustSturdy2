// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.vision.Vision;

public class AprilTagMovement extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  public static double initialHeading;
  /** Creates a new AprilTag. */
  public AprilTagMovement(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  double tP = 0.01145;
  double tI = 0.0000176;
  double tD = 0.00098;
  PIDController aprilTagPID = new PIDController(tP, tI, tD);
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    initialHeading = driveSubsystem.getHeading();
    SmartDashboard.putNumber("Heading", initialHeading);

    if (Vision.tV == true && Vision.targetID == 8) {
      double tX = Vision.tX;
      double turnValue = aprilTagPID.calculate(tX, 0);
      SmartDashboard.putNumber("TurnValue", turnValue);
      driveSubsystem.drive(0, 0, -turnValue, false, true);
      
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
