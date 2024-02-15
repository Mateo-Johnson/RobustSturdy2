// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.DriveConstants;

public class PurgeRing extends CommandBase {
  /** Creates a new Purge. */
  public static final CANSparkMax intake1 = DriveConstants.rightIntake;
  public static final CANSparkMax intake2= DriveConstants.leftIntake;
  public PurgeRing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runIntake(-0.8);
  }

    //FUNCTIONS FOR SIMPLICITY
    public void runIntake(double speed) {
      intake1.set(speed);
      intake2.set(speed);
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
}
