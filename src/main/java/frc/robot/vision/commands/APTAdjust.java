// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class APTAdjust extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  public static double initialHeading;
  /** Creates a new APTAdjust. */
  public APTAdjust(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  //PID VALUES
  double tP = 0.01145;
  double tI = 0.0000176;
  double tD = 0.00098;
  //CREATE A PID CONTROLLER WITH THE SPECIFIED CONSTANTS
  PIDController turningPID = new PIDController(tP, tI, tD);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  double tX = tx.getDouble(0.0);
  double tY = ty.getDouble(0.0);
  double target = tv.getDouble(0.0);

  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double tX = tx.getDouble(0.0);

      if (target == 1) {
      //THE LINE BELOW BASICALLY MEANS THAT IT IS CALCULATING THE PID CONTROLLER VALUE, TRYING TO MAKE THE FIRST VALUE MATCH THE SECOND VALUE
      double turnValue = turningPID.calculate(tX, 0); //CREATE THE PID CONTROLLER, FROM THE STARTING POINT OF THE X OFFSET, AND MOVING TO ZERO
      SmartDashboard.putNumber("TurnValue", turnValue);
      driveSubsystem.drive(0, 0, turnValue, false, true);
      }
      
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
