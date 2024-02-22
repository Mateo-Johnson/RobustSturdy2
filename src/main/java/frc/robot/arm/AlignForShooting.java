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

  static AbsoluteEncoder armEncoder = Arm.armEncoder;

  private static final int cyclesPerRotation = 2048;
  private static final int drivenGearTeeth = 60;
  private static final int driveGearTeeth = 15;


  public AlignForShooting(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    initialHeading = driveSubsystem.getHeading();
  }

  @Override
  public void initialize() {}

  public static double armEncoderReading =  (armEncoder.getPosition() - 0.42638435959816) * -1;
  public static double gearRatio = (double) drivenGearTeeth / driveGearTeeth;
  public static int encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio);
  public static double degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution; // Corrected line
  public static double degrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tX = tx.getDouble(0.0); 
    double tY = ty.getDouble(0.0); 
    double tA = ta.getDouble(0.0); 

    double armEncoderReading =  (armEncoder.getPosition() - 0.42638435959816) * -1;

    double knownDistance = 12.0;  // known distance in feet
    double knownTA = 0.160;  // known tA value at the known distance
    double currentTA = tA;  // current tA value

    double distance = calculateDistance(currentTA, knownTA, knownDistance);

    double targetAngle = Math.atan(distance/70);

    SmartDashboard.putNumber("so silly", distance);
    SmartDashboard.putNumber("so sillyer", targetAngle);

    //ENCODER TRANSLATING
    double gearRatio = (double) drivenGearTeeth / driveGearTeeth;
    int encoderCyclesPerArmRevolution = (int) (cyclesPerRotation * gearRatio);
    double degreesPerEncoderCycle = 360.0 / encoderCyclesPerArmRevolution; // Corrected line
    double degrees = armEncoderReading * encoderCyclesPerArmRevolution * degreesPerEncoderCycle;
    SmartDashboard.putNumber("idk man", degrees);

    //CODE FOR HOLDING THE ARM IN PLACE
    SmartDashboard.putNumber("arm angle", armEncoderReading);
    // double armSetpoint = targetAngle;

    //PID LOOPS
    // double turnValue = armMovePID.calculate(degrees, armSetpoint);
    double armValue = armAlignPID.calculate(tY, 2);
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

  public static double calculateDistance(double currentTA, double knownTA, double knownDistance) {
    // Calculate the ratio of current and known tA values
    double tA_ratio = currentTA / knownTA;

    // Adjust the distance based on the ratio
    double distance = knownDistance / tA_ratio;

    return distance;
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
