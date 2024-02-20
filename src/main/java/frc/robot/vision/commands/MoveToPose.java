// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveSubsystem;

public class MoveToPose extends CommandBase {
  /** Creates a new MoveToPose. */
  public MoveToPose(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    
  }

    //PID VALUES
    double dP = 0.0001;
    double dI = 0;
    double dD = 0;
    //CREATE A PID CONTROLLER WITH THE SPECIFIED CONSTANTS
    PIDController turningPID = new PIDController(dP, dI, dD);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
