// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.DriveConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class IntakeRing extends CommandBase {
  /** Creates a new silly. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color BlueTarget = new Color(0.143, 0.427, 0.429); //CREATES A COLOR
  private final Color GreenTarget = new Color(0.197, 0.561, 0.240); //CREATES A COLOR
  private final Color RedTarget = new Color(0.561, 0.232, 0.114); //CREATES A COLOR
  private final Color YellowTarget = new Color(0.361, 0.524, 0.113); //CREATES A COLOR
  private final Color OrangeTarget = new Color(0.453, 0.405, 0.141); //CREATES A COLOR
  public static boolean hasRing;

  public IntakeRing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  //INTAKE
  public static final CANSparkMax intake1 = DriveConstants.rightIntake;
  public static final CANSparkMax intake2 = DriveConstants.leftIntake;


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorMatcher.addColorMatch(BlueTarget);
    colorMatcher.addColorMatch(GreenTarget);
    colorMatcher.addColorMatch(RedTarget);
    colorMatcher.addColorMatch(YellowTarget); 
    colorMatcher.addColorMatch(OrangeTarget);
    hasRing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Color detectedColor = colorSensor.getColor();
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == BlueTarget) {
      colorString = "blue";
      ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE

    } else if (match.color == RedTarget) {
      colorString = "red";
      stopIntake();
      ringInIntake(true); //FLAG TO MARK WHEN THERE IS A RING IN INTAKE - 

    } else if (match.color == GreenTarget) {
      colorString = "green";
      ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE

    } else if (match.color == YellowTarget) {
      colorString = "yellow";
      runIntake(0.4);
      ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE 

    } else if (match.color == OrangeTarget){
      colorString = "orange";
      stopIntake();
      ringInIntake(true); //FLAG TO MARK WHEN THERE IS A RING IN INTAKE - 

    } else {
      colorString = "we can't tell";
      runIntake(0.4);
      ringInIntake(false); //FLAG TO MARK WHEN THERE ISN'T A RING IN INTAKE
    }

    SmartDashboard.putNumber("red value", detectedColor.red);
    SmartDashboard.putNumber("green value", detectedColor.green);
    SmartDashboard.putNumber("blue value", detectedColor.blue);
    SmartDashboard.putNumber("confidence", match.confidence);
    SmartDashboard.putString("detected color", colorString);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stopIntake();
    hasRing = false;
  }



  //FUNCTIONS FOR SIMPLICITY
  public void runIntake(double speed) {
    intake1.set(speed);
    intake2.set(-speed);
  }

  public void stopIntake(){
    intake1.set(0);
    intake2.set(0);
  }

  public void ringInIntake(boolean trueslashfalse) {
    hasRing = trueslashfalse;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


