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
  public static final CANSparkMax OUTR = DriveConstants.leftOuttake;
  public static final CANSparkMax OUTL = DriveConstants.rightOuttake;
  public static final CANSparkMax INR = DriveConstants.rightIntake;
  public static final CANSparkMax INL = DriveConstants.leftIntake;
  private RelativeEncoder shooterEncoder = OUTL.getEncoder();




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("silly", shooterEncoder.getVelocity());
    if (shooterEncoder.getVelocity() >= 19) { //IF THE MOTORS ARE SPINNING FAST ENOUGH 
      OUTL.set(5); //SET UP OUTTAKE MOTOR 1 FOR SHOOTING
      OUTR.set(5); //SET UP OUTTAKE MOTOR 2 FOR SHOOTING
      INL.set(-0.5); //USE INTAKE MOTOR 1 TO FEED INTO OUTTAKE
      INR.set(0.5); //USE INTAKE MOTOR 2 TO FEED INTO OUTTAKE
      //ADD A METHOD MAKE THE BOTTOM LIGHTS GREEN TO SHOW THAT ITS READY TO SHOOT

    } else if (shooterEncoder.getVelocity() <= 19) { //IF THE MOTORS ARE NOT AT THE RIGHT SPEED
      OUTL.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      OUTR.set(5); //MAKE OUTTAKE MOTOR 1 GO TO RIGHT SPEED
      //ADD A METHOD MAKE THE BOTTOM LIGHTS RED TO SHOW THAT ITS NOT READY TO SHOOT
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    OUTR.set(0);
    OUTL.set(0);
    INR.set(0);
    INL.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
