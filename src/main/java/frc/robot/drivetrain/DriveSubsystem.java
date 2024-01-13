//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot.drivetrain;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  //CREATE SWERVE MODULES


  //THIS IS THE FRONT LEFT MODULE
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.frontLeftDrivingCanId,
      DriveConstants.frontLeftTurningCanId,
      DriveConstants.frontLeftChassisAngularOffset);


  //THIS IS THE FRONT RIGHT MODULE
  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.frontRightDrivingCanId,
      DriveConstants.frontRightTurningCanId,
      DriveConstants.frontRightChassisAngularOffset);


  //THIS IS THE BACK LEFT MODULE
  private final SwerveModule rearLeft = new SwerveModule(
      DriveConstants.rearLeftDrivingCanId,
      DriveConstants.rearLeftTurningCanId,
      DriveConstants.backLeftChassisAngularOffset);


  //THIS IS THE BACK RIGHT MODULE
  private final SwerveModule rearRight = new SwerveModule(
      DriveConstants.rearRightDrivingCanId,
      DriveConstants.rearRightTurningCanId,
      DriveConstants.backRightChassisAngularOffset);


  //GYRO THIS IS WHERE THE GYRO GOES (CURRENTLY THIS IS SAYING THAT IT IS A NAVX MOUNTED TO THE TOP PART OF THE RIO)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);


  //I DONT EVEN KNOW WHAT SLEW RATE IS BUT THESE CONTROL SLEW RATE AND THE DOCS TOLD ME TO (LATERAL MOVEMENT MAYBE?)
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.magnitudeSlewRate);   //LIMITS THE RATE OF CHANGE OF THE MAGNITUDE OF THE ROBOT'S SPEED
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.rotationalSlewRate);  //LIMITS THE RATE OF CHANGE OF THE ROTATION SPEED OF THE ROBOT
  private double prevTime = WPIUtilJNI.now() * 1e-6;


  //TRACKING ROBOT POSE
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });


  //THIS IS THE CHOOSER FOR THE AUTO OPTIONS
  private SendableChooser<Command> autoChooser;


  //CREATES A NEW DRIVESUBSYSTEM.
  public DriveSubsystem() {


    AutoBuilder.configureHolonomic(
      this::getPose, //POSE SUPPLIER
      this::resetOdometry, //METHOD TO RESET ODOMETRY
      () -> DriveConstants.DriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
      ),
      (speeds) -> drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false),
      new HolonomicPathFollowerConfig( //HOLONOMIC PATH FOLLOWER CONFIG
          new PIDConstants(5.0, 0.0, 0.0), //TRANSLATION PID CONSTANTS
          new PIDConstants(5.0, 0.0, 0.0), //ROTATION PID CONSTANTS
          4.5, //MAX SPEED IN M/S
          0.4, //DISTANCE FROM CENTER TO FURTHEST MODULE
          new ReplanningConfig() //DEFAULT PATH REPLANNING CONFIG
      ),
      null, this //REFERENCE TO THIS SUBSYSTEM TO SET REQUIREMENTS
    );


    // //THIS CREATES THE CHOICES FOR AUTOS AND PUSHES THEM TO SMARTDASHBOARD
    // autoChooser = new SendableChooser<>();
    // autoChooser = AutoBuilder.buildAutoChooser(); //USES COMMANDS.NONE AS THE DEFAULT OPTION
    // //THE CODE BELOW IS A OPTION THAT ALLOWS YOU TO SPECIFY THE DEFAULT AUTO BY ITS NAME
    // //autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");


    // SmartDashboard.putData("Auto Chooser", autoChooser); //SEND THE DATA TO SMARTDASHBOARD


  }




  @Override
  public void periodic() {
    //UPDATE ODOMETRY
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
  }


  /**
   * RETURNS THE ROBOT POSE
   *
   * @return THE POSE
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }


  /**
   * RESETS ODOMETRY TO SPECIFIED POSE
   *
   * @param pose THE POSE TO SET ODOMETRY TO
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }


  /**
   * DRIVE ROBOT USING JOYSTICK INPUT
   *
   * @param xSpeed        SPEED OF THE ROBOT IN THE X DIRECTION (FORWARD/BACK)
   * @param ySpeed        SPEED OF THE ROBOT IN THE Y DIRECTION (SIDEWAYS)
   * @param rot           ANGULAR RATE OF THE ROBOT
   * @param fieldRelative WHETHER THE PROVIDED SPEEDS ARE ROBOT RELATIVE
   * @param rateLimit     WHETHER TO ENABLE RATE LIMITING FOR EASIER CONTROL
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
   
    double xSpeedCommanded;
    double ySpeedCommanded;


    if (rateLimit) {
      //CONVERT XY VALUES TO POLAR FOR RATE LIMITING
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


      //CALCULATE LEW RATE BASED ON ESTIMATE OF LATERAL ACCELERATION
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.directionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //HIGH NUMBER TO MAKE IT ALMOST INSTANTANEOUS
      }
     


      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //TINY NUMBER TO AVOID FLOATING POINT ERRORS
          //KEEP CURRENT TRANSLATION DIR THE SAME
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
     
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);




    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }


    //CONVERT COMMAND SPEEDS INTO DRIVETRAIN READY SPEEDS
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.maxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.maxAngularSpeed;


    var swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }


  //SET WHEELS INTO X FORMATION TO STOP MOVEMENT
  public void setWheelsX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates THE DESIRED MODULE STATES
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }


  //RESETS ENCODERS TO READ 0
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }


  //ZEROS HEADING OF ROBOT
  public void zeroHeading() {
    gyro.reset();
  }


  /**
   * RETURNS THE HEADING
   *
   * @return THE ROBOT HEADING (-180 to 180)
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }


  /**
   * RETURNS THE TURN RATE
   *
   * @return THE TURN RATE FOR THE ROBOT IN DEGREES PER SECOND
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }
}



