//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//WPILib BSD license file in the root directory of this project.


package frc.robot.utils;


import com.revrobotics.CANSparkMax.IdleMode;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    //THIS IS HOW FAR THE ACTUAL ANGLE CAN BE FROM THE EST. ANGLE WITHOUT IT GETTING ANGRY
    public static final double turnToleranceDegrees = 2.0;
    public static double translationToleranceMeters = 2.0;
    //DRIVING PARAMS - MAX CAPABLE SPEEDS NOT MAX ALLOWED SPEEDS
    public static final double maxSpeedMetersPerSecond = 4.8;
    public static final double maxAngularSpeed = 2 * Math.PI; //RADIANS PER SECOND


    public static final double directionSlewRate = 1.2; //PERCENT PER SECOND (PPS) (1=100%)
    public static final double magnitudeSlewRate = 1.8; //PERCENT PER SECOND (PPS) (1=100%)
    public static final double rotationalSlewRate = 2.0; //PERCENT PER SECOND (PPS) (1=100%)


    //CHASSIS CONFIG
    public static final double trackWidth = Units.inchesToMeters(30);
    //DISTANCE BETWEEN CENTERS OF RIGHT AND LEFT WHEELS
    public static final double wheelBase = Units.inchesToMeters(28);
    //DISTANCE BETWEEN FRONT AND BACK WHEELS
    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));


    //ANGULAR OFFSETS OF THE MODULES RELATIVES TO THE CHASSIS IN RADIANS
    //MATEO YOU NEED TO CHANGE THESE THINGS FIRST THING WEDNESDAY
    public static final double frontLeftChassisAngularOffset = 2.33; //GOOD OFFSET
    public static final double frontRightChassisAngularOffset = 4.99; //GOOD OFFSET
    public static final double backLeftChassisAngularOffset = 0.71; //GOOD OFFSET
    public static final double backRightChassisAngularOffset = 2.65; //GOOD OFFSET
 



    //SPARK MAX CAN IDS


    //FRONT LEFT MODULE
    public static final int frontLeftDrivingCanId = 8;
    public static final int frontLeftTurningCanId = 9;
    //BACK LEFT MODULE
    public static final int rearLeftDrivingCanId = 5;
    public static final int rearLeftTurningCanId = 4;
    //FRONT RIGHT MODULE
    public static final int frontRightDrivingCanId = 7;
    public static final int frontRightTurningCanId = 6;
    //BACK RIGHT MODULE
    public static final int rearRightDrivingCanId = 3;
    public static final int rearRightTurningCanId = 2;
    //IS THE GYRO REVERSED??????
    public static final boolean gyroReversed = false;


   
  }


  public static final class ModuleConstants {


    public static final int low = 12; //LOW SPEED PINION GEAR
    public static final int medium = 13; //mID SPEED PINION GEAR
    public static final int high = 14; //HIGH SPEED PINION GEAR


    //THIS CAN BE 12, 13 OR 14 CONSULT MECHANICAL BEFORE CHANGING, ASK FOR MODULE PINION TEETH NUMBER (IDEALLY ASK AHMED A.)
    public static final int drivingMotorPinionTeeth = medium;


    //OUTPUT SHAFT ROTATES OPPOSITE OF STEERING MOTOR SO INVERT
    public static final boolean turningEncoderInverted = true;


    //CALCULATIONS FOR DRIVE MOTOR CONVERSION FACTORS AND FEED FORWARD
    public static final double drivingMotorFreeSpeedRps = NeoMotorConstants.freeSpeedRpm / 60;
    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    //45 TEETH ON BEVEL, 22 TEETH ON FIRST-STAGE SPUR, 15 TEETH ON BEVEL PINION
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
        / drivingMotorReduction;


    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI)
        / drivingMotorReduction; //METERS
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI)
        / drivingMotorReduction) / 60.0; //METERS PER SECOND


    public static final double turningEncoderPositionFactor = (2 * Math.PI); //RADIANS
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; //RADIANS PER SECOND


    public static final double turningEncoderPositionPIDMinInput = 0; //RADIANS
    public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor; //RADIANS


    public static final double drivingP = 0.04;
    public static final double drivingI = 0;
    public static final double drivingD = 0;
    public static final double drivingFF = 1 / driveWheelFreeSpeedRps;
    public static final double drivingMinOutput = -1;
    public static final double drivingMaxOutput = 1;


    public static final double turningP = 1;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningFF = 0;
    public static final double turningMinOutput = -1;
    public static final double turningMaxOutput = 1;


    public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode turningMotorIdleMode = IdleMode.kBrake;




    public static final int drivingMotorCurrentLimit = 50; //AMPS
    public static final int turningMotorCurrentLimit = 20; //AMPS
  }


  public static final class ControllerConstants {
    public static final int driverControllerPort = 0; //PRIMARY DRIVER PORT
    public static final int secondaryControllerPort = 1; //SECONDARY DRIVER PORT
    public static final double driveDeadzone = 0.2; //DEADZONE OF JOYSTICKS
  }


  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;


    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;


    //CONSTRAINTS FOR MOTION PROFILED ROBOT ANGLE CONTROLLER
    public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }


  public static final class NeoMotorConstants {
    public static final double freeSpeedRpm = 5676;
  }
}





