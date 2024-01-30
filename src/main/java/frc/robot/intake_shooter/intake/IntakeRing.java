// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake_shooter.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.DriveConstants;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

public class IntakeRing extends CommandBase {
  /** Creates a new IntakeRing. */
  private final ColorSensorV3 colorSensor = DriveConstants.colorSensor; //CREATES THE COLOR SENSOR CONNECTED TO THE ABOVE PORT (ALL IN DRIVECONSTANTS)
  private final ColorMatch colorMatcher = new ColorMatch(); //CREATES A NEW COLOR MATCHER (USED TO REGISTER AND DETECT KNOWN COLORS)
  private final Color orange = new Color(229, 34, 0); //CREATES A COLOR

  public static CANSparkMax rightIntake = new CANSparkMax(DriveConstants.rightIntakeCanId, MotorType.kBrushless);
  public static CANSparkMax leftIntake = new CANSparkMax(DriveConstants.leftIntakeCanId, MotorType.kBrushless);


  public IntakeRing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorMatcher.addColorMatch(orange); //ADD A COLOR FOR REFERENCE TO THE COLOR MATCHER
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Color detectedColor = colorSensor.getColor(); //DETECT THE COLOR FROM THE COLOR SENSOR
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    colorSensor.getRawColor();
    colorMatcher.matchClosestColor(detectedColor);

    if (match.color != orange) {
      runIntake(1);
    } else if (match.color == orange) {
      runIntake(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public void runIntake(double speed) {
    rightIntake.set(speed);
    leftIntake.set(-speed);
  }

}


