// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveSubsystem;

public class AlignForShooting extends CommandBase {
  /** Creates a new MoveArm. */
  public static double initialHeading;
  private final DriveSubsystem driveSubsystem;
  double aP = 0.01;
  double aI = 0.0;
  double aD = 0.0;
  // double aP = 0.8;
  // double aI = 0.1;
  // double aD = 0.51;
  PIDController armPID = new PIDController(aP, aI, aD);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  double tX = tx.getDouble(0.0);
  double tY = ty.getDouble(0.0);
  double target = tv.getDouble(0.0);

    //PID VALUES
    double tP = 0.01145;
    double tI = 0.0000176;
    double tD = 0.00098;
    //CREATE A PID CONTROLLER WITH THE SPECIFIED CONSTANTS
    PIDController turningPID = new PIDController(tP, tI, tD);

  public AlignForShooting(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // AbsoluteEncoder armEncoder = Arm.armEncoder;
    // double armEncoderReading = armEncoder.getPosition();
    // double armSetpoint = armEncoderReading;
    // double turnValue = armPID.calculate(armEncoderReading, armSetpoint);
    // Arm.leftArm.set(turnValue * 5);
    // Arm.rightArm.set(-turnValue * 5);
    // SmartDashboard.putNumber("Turn Value", turnValue);
    double tY = ty.getDouble(0.0);
    double turnValue = armPID.calculate(tY, 5);
    Arm.leftArm.set(-turnValue * 2);
    Arm.rightArm.set(turnValue * 2);

    double tX = tx.getDouble(0.0);

    //THE LINE BELOW BASICALLY MEANS THAT IT IS CALCULATING THE PID CONTROLLER VALUE, TRYING TO MAKE THE FIRST VALUE MATCH THE SECOND VALUE
    double turnValue1 = turningPID.calculate(tX, 0); //CREATE THE PID CONTROLLER, FROM THE STARTING POINT OF THE X OFFSET, AND MOVING TO ZERO
    SmartDashboard.putNumber("TurnValue", turnValue);
    driveSubsystem.drive(0, 0, turnValue1, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.leftArm.set(0);
    Arm.rightArm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
