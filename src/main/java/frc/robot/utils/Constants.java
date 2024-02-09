//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//WPILib BSD license file in the root directory of this project.


package frc.robot.utils;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;


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
  public static final class FieldConstants {
    //DEFINE FIELD DIMENSIONS
    public static final double field_length = 54.0; //FEET
    public static final double field_width = 27.0; //FEET
    //DEFINE ORIGIN POSITION
    public static final double origin_y = 0.0;
    public static final double origin_x = 0.0;

  }

  public static final class DriveConstants {

    //TIME THINGS
    public static long currentTimeMillis = System.currentTimeMillis(); //THE TIME IN MILLISECONDS
    public static long currentTimeSeconds = currentTimeMillis / 1000; //THE TIME IN SECONDS

    //THIS IS HOW FAR THE ACTUAL ANGLE CAN BE FROM THE EST. ANGLE WITHOUT IT GETTING ANGRY
    public static final double turnToleranceDegrees = 2.0;
    public static double translationToleranceMeters = 2.0;
    //DRIVING PARAMS - MAX CAPABLE SPEEDS NOT MAX ALLOWED SPEEDS
    public static final double maxSpeedMetersPerSecond = 4.8;
    public static final double maxAngularSpeed = 2 * Math.PI; //RADIANS PER SECOND


    public static final double directionSlewRate = 1.2; //PERCENT PER SECOND (PPS) (1=100%)
    public static final double magnitudeSlewRate = 1.8; //PERCENT PER SECOND (PPS) (1=100%)
    public static final double rotationalSlewRate = 2.0; //PERCENT PER SECOND (PPS) (1=100%)

    public static final double encCountsPerRev = 4096;
    public static final double wheelDiamIn = 3;


    //CHASSIS CONFIG
    public static final double trackWidth = Units.inchesToMeters(30);
    //DISTANCE BETWEEN CENTERS OF RIGHT AND LEFT WHEELS ↑
    public static final double wheelBase = Units.inchesToMeters(28);
    //DISTANCE BETWEEN FRONT AND BACK WHEELS ↑ 🤪
    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));


    //ANGULAR OFFSETS OF THE MODULES RELATIVES TO THE CHASSIS IN RADIANS
    public static final double frontLeftChassisAngularOffset = 2.33; //GOOD OFFSET
    public static final double frontRightChassisAngularOffset = 4.99; //GOOD OFFSET
    public static final double backLeftChassisAngularOffset = 0.71; //GOOD OFFSET
    public static final double backRightChassisAngularOffset = 2.65; //GOOD OFFSET
 


    //---------------------DECLARATIONS FOR PHYSICAL PLACEMENTS/WIRINGS OF THINGS-----------------------------//
    /*EYO THIS IS THE KEY FOR THE MOTOR SPARKMAXES IF YOU NEED THEM THEY SHOULD ALL BE RIGHT HERE
     * MOTORS
     * 2 - REAR RIGHT SWERVE TURNING (NEEDS ENCODER)
     * 3 - REAR RIGHT SWERVE DRIVING
     * 4 - REAR LEFT SWERVE TURNING (NEEDS ENCODER)
     * 5 - REAR LEFT SWERVE DRIVING
     * 6 - FRONT RIGHT SWERVE TURNING (NEEDS ENCODER)
     * 7 - FRONT RIGHT SWERVE DRIVING
     * 8 - FRONT LEFT SWERVE DRIVING
     * 9 - FRONT LEFT SWERVE TURNING (NEEDS ENCODER)
     * 
     * 11 - LEFT ARM ROTATION MOTOR 
     * 12 - RIGHT ARM ROTATION MOTOR
     * 
     * 21 - LEFT ARM INTAKE MOTOR
     * 22 - RIGHT ARM INTAKE MOTOR
     * 23 - LEFT ARM OUTTAKE MOTOR
     * 24 - RIGHT ARM OUTTAKE MOTOR
    */
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
    //RIGHT AND LEFT NEO 55OS FOR INTAKE
    public static final int rightIntakeCanId = 11;
    public static final int leftIntakeCanId = 12;
    //RIGHT AND LEFT UNGEARBOXED NEOS FOR OUTTAKE
    public static final int rightOuttakeCanId = 23;
    public static final int leftOuttakeCanId = 24;
    //RIGHT AND LEFT TORQUE GEARBOXED NEOS FOR ARM MOVEMENT
    public static final int leftArmMotorCanId = 35;
    public static final int rightArmMotorCanId = 36;

    public static CANSparkMax rightIntake = new CANSparkMax(DriveConstants.rightIntakeCanId, MotorType.kBrushless);    
    public static CANSparkMax leftIntake = new CANSparkMax(DriveConstants.leftIntakeCanId, MotorType.kBrushless); 
    public static CANSparkMax rightOuttake = new CANSparkMax(DriveConstants.rightOuttakeCanId, MotorType.kBrushless);    
    public static CANSparkMax leftOuttake = new CANSparkMax(DriveConstants.leftOuttakeCanId, MotorType.kBrushless); 
    public static CANSparkMax leftArm = new CANSparkMax(DriveConstants.leftArmMotorCanId, MotorType.kBrushless);    
    public static CANSparkMax rightArm = new CANSparkMax(DriveConstants.rightArmMotorCanId, MotorType.kBrushless); 

    public static final AbsoluteEncoder armEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);

    //THE I2C PORT FOR THE COLOR SENSOR
    public static final I2C.Port I2CPort = I2C.Port.kOnboard;
    public static final ColorSensorV3 colorSensor = new ColorSensorV3(I2CPort);

  }


  public static final class ModuleConstants {


    public static final int low = 12; //LOW SPEED PINION GEAR
    public static final int medium = 13; //mID SPEED PINION GEAR
    public static final int high = 14; //HIGH SPEED PINION GEAR


    //THIS CAN BE 12, 13 OR 14 CONSULT MECHANICAL BEFORE CHANGING, ASK FOR MODULE PINION TEETH NUMBER (IDEALLY ASK AHMED A.)
    public static final int drivingMotorPinionTeeth = medium;

    public static final boolean so_true = true;
    //OUTPUT SHAFT ROTATES OPPOSITE OF STEERING MOTOR SO INVERT
    public static final boolean turningEncoderInverted = so_true;


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


    public static final double drivingP = 0.01;
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
    public static final double freeSpeedRpm = 5676; //MATEO REMEMBERS THIS OFF THE TOP OF HIS HEAD LMAO
  }
}







