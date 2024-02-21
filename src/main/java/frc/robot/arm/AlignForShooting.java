// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// JUST AS A NOTE, THE ARM RESTS AT ABOUT 0.422 ABS ENCODER READING

package frc.robot.arm;

import com.revrobotics.AbsoluteEncoder;

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

  //PID VALUES FOR MOVING THE ARM
  double aP = 0.01;
  double aI = 0.0;
  double aD = 0.0;

  //PID VALUES BUT THEY'RE DIFFERENT FOR ARM
  double aMP = 0.8;
  double aMI = 0.1;
  double aMD = 0.51;

  //PID VALUES FOR TURNING
  double tP = 0.01145;
  double tI = 0.0000176;
  double tD = 0.00098;

  //CREATE THE ARM AND TURNING PID SYSTEMS
  PIDController armAlignPID = new PIDController(aP, aI, aD);
  PIDController turningPID = new PIDController(tP, tI, tD);
  PIDController armMovePID = new PIDController(aMP, aMI, aMD);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
  NetworkTableEntry tx = table.getEntry("tx");//THE X OFFSET OF THE TARGET IN THE CAMERA VIEW
  NetworkTableEntry ty = table.getEntry("ty"); //THE Y OFFSET OF THE TARGET IN THE CAMERA VIEW
  NetworkTableEntry ta = table.getEntry("ta"); //THE AREA THAT THE TARGET TAKES UP ON THE SCREEN
  NetworkTableEntry tv = table.getEntry("tv"); //GET WHETHER THE LIMELIGHT HAS A TARGET OR NOT (1 OR 0)

  double tX = tx.getDouble(0.0); //SET tx = tX AND SET THE DEFAULT VALUE TO 0
  double tY = ty.getDouble(0.0); //SET ty = tY AND SET THE DEFAULT VALUE TO 0
  double tA = ta.getDouble(0.0); //SET ta = tA AND SET THE DEFAULT VALUE TO 0
  double tV = tv.getDouble(0.0); //SET tv = tV AND SET THE DEFAULT VALUE TO 0

  public AlignForShooting(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //CODE FOR HOLDING THE ARM IN PLACE
    AbsoluteEncoder armEncoder = Arm.armEncoder;
    double armEncoderReading = armEncoder.getPosition();
    SmartDashboard.putNumber("arm angle", armEncoderReading);
    double armSetpoint = 0.259;
    double tY = ty.getDouble(0.0);
    double tX = tx.getDouble(0.0);

    double turnValue = armMovePID.calculate(armEncoderReading, armSetpoint);
    double armValue = armAlignPID.calculate(tY, 5);
    double turnValue1 = turningPID.calculate(tX, 0);

    //MOVE THE ARM TO THE SPECIFIC VALUE ABOVE THE APRILTAG
    PIDMoveArm(armValue);
    //ALIGN THE DRIVETRAIN TO THE APRILTAG
    driveSubsystem.drive(0, 0, turnValue1, false, true);


      // //MOVE THE ARM TO THE RIGHT POSITION
      // Arm.leftArm.set(turnValue * 5);
      // Arm.rightArm.set(-turnValue * 5);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.leftArm.set(0);
    Arm.rightArm.set(0);
  }

  //FUNCTION FOR MOVING THE ARM
  public void PIDMoveArm(double angle) {
    Arm.leftArm.set(-angle * 2);
    Arm.rightArm.set(angle * 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
