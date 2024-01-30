// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake_shooter.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.DriveConstants;

public class ShootRingIndiscriminately extends CommandBase {
  /** Creates a new ShootRing. */

  public static CANSparkMax rightOuttake = new CANSparkMax(DriveConstants.rightOuttakeCanId, MotorType.kBrushless);
  public static CANSparkMax leftOuttake = new CANSparkMax(DriveConstants.leftOuttakeCanId, MotorType.kBrushless);
  
  public ShootRingIndiscriminately() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runOuttake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void runOuttake(double speed) {
    rightOuttake.set(speed);
    leftOuttake.set(-speed);
  }

}
