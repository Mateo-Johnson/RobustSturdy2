// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ModuleConstants;

public class TurnToAngle extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  public static double initialHeading;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  double tP = 0.01145;
  double tI = 0.0000176;
  double tD = 0.00098;
  PIDController turningPID = new PIDController(tP, tI, tD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    initialHeading = driveSubsystem.getHeading();
    SmartDashboard.putNumber("Heading", initialHeading);
    double turnValue = turningPID.calculate(initialHeading, 90);
    SmartDashboard.putNumber("TurnValue", turnValue);
    driveSubsystem.drive(0, 0, -turnValue, false, true);


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
