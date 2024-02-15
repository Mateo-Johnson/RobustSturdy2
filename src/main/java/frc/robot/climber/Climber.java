package frc.robot.climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants.DriveConstants;


public class Climber extends CommandBase {
  /** Creates a new IntakeRing. */
  Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, DriveConstants.rightSolenoidChannelID);
  Solenoid wrongSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, DriveConstants.wrongSolenoidChannelID);
  Compressor compressor = DriveConstants.compressor;

  double compressorCurrent = DriveConstants.compressorCurrent; //CURRENT CURRENT DRAW (see what i did there)
  boolean compressorOn = DriveConstants.compressorOn; //IS THE COMPRESSOR ON
  boolean compressorPressure = DriveConstants.compressorPressure; //RETURNS TRUE WHEN THE COMPRESSOR IS FULL

  public Climber() {
    // Use addRequirements() here to declare subsystem dependencies.

    //BASICS OF SOLENOIDS: BASICALLY, THE COMPRESSOR TAKES IN  AIR AND COMPRESSES IT. THE SOLENOIDS ARE LIKE LITTLE GATES, 
    //IF THEY ARE SET TO FALSE THEN THEY DON'T LET AIR THROUGH. THE PISTONS ARE JUST POWERED BY THAT AIR.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (compressorOn == true && compressorPressure == true) { //IF THE COMPRESSOR IS ON AND THE TANK IS FULL
      rightSolenoid.set(true);
      wrongSolenoid.set(true);
      compressor.disable();
    } else { //IN ANY OTHER SITUATION
      rightSolenoid.set(false);
      wrongSolenoid.set(false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rightSolenoid.set(false);
    wrongSolenoid.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}



