// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.DriveConstants;

public class PurgeRing extends CommandBase {
  /** Creates a new Purge. */
  public static final CANSparkMax rightIntake = DriveConstants.rightIntake;
  public static final CANSparkMax wrongIntake= DriveConstants.leftIntake;
  public static final CANSparkMax rightOuttake = DriveConstants.rightOuttake;
  public static final CANSparkMax wrongOuttake = DriveConstants.leftOuttake;
  public PurgeRing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runIntakeAndOuttake(-0.8);
  }

    //FUNCTIONS FOR SIMPLICITY
    public void runIntakeAndOuttake(double speed) {
      rightIntake.set(speed);
      wrongIntake.set(-speed);
      rightOuttake.set(speed);
      wrongOuttake.set(-speed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runIntakeAndOuttake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
