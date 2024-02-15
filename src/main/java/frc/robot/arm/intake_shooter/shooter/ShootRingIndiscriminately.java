// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.DriveConstants;

public class ShootRingIndiscriminately extends CommandBase {
  /** Creates a new silly. */
  public ShootRingIndiscriminately() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  //OUTTAKE
  public static final CANSparkMax OUT1 = DriveConstants.leftOuttake;
  public static final CANSparkMax OUT2 = DriveConstants.rightOuttake;
  public static final CANSparkMax IN1 = DriveConstants.rightIntake;
  public static final CANSparkMax IN2 = DriveConstants.leftIntake;
  private RelativeEncoder shooterEncoder = OUT1.getEncoder();




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("silly", shooterEncoder.getVelocity());
    if (shooterEncoder.getVelocity() >= 20) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
      OUT1.set(5); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING
      OUT2.set(5); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING
      IN1.set(0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
      IN2.set(0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
      //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT

    } else if (shooterEncoder.getVelocity() <= 20) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
      OUT1.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      OUT2.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    OUT1.set(0);
    OUT2.set(0);
    IN1.set(0);
    IN2.set(0);
  }

  public void silly() {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
