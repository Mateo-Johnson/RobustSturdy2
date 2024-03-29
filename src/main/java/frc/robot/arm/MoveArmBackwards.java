// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmBackwards extends CommandBase {
  /** Creates a new MoveArmBackwards. */
  public MoveArmBackwards() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (AlignForShooting.degrees >= 0) {

    Arm.rotateVector(0.3);
    
    }
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
