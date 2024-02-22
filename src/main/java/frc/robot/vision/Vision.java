package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.LimelightLib;

public class Vision extends SubsystemBase {

public static String limelightName = "limelight";

//THE LIMELIGHT URL

//COORDINATE VALUE GETTERS
public static double tX = LimelightLib.getTX(limelightName);
public static double tY = LimelightLib.getTY(limelightName);
public static double tA = LimelightLib.getTA(limelightName);
public static boolean tV = LimelightLib.getTV(limelightName);

//ROBOT POSE GETTERS 
//3D
public static Pose3d botPose3d = LimelightLib.getBotPose3d(limelightName);
public static double[] botPoseTargetSpace = LimelightLib.getBotPose_TargetSpace(limelightName);
public static double[] botPose3dBlue = LimelightLib.getBotPose_wpiBlue(limelightName);
public static double[] botPose3dRed = LimelightLib.getBotPose_wpiRed(limelightName);
//2D
public static double[] botPose = LimelightLib.getBotPose(limelightName);
public static Pose2d botPose2d = LimelightLib.getBotPose2d(limelightName);
public static Pose2d botPose2dBlue = LimelightLib.getBotPose2d_wpiBlue(limelightName);
public static Pose2d botPose2dRed = LimelightLib.getBotPose2d_wpiRed(limelightName);

//CAMERA POSE GETTERS
public static Pose3d camPoseRobotSpace = LimelightLib.getCameraPose3d_RobotSpace(limelightName);
public static Pose3d camPose3dRobotSpace = LimelightLib.getCameraPose3d_TargetSpace(limelightName);
public static double[] camPose2dRobotSpace = LimelightLib.getCameraPose_TargetSpace(limelightName);

//TARGET INFO GETTERS
public static Pose3d targetPose3d = LimelightLib.getTargetPose3d_RobotSpace(limelightName);
public static double targetID = LimelightLib.getFiducialID(limelightName);
public static double[] targetColor = LimelightLib.getTargetColor(limelightName);

// Extract the position of the target from the Pose3d object
public static Translation3d targetPosition = targetPose3d.getTranslation();

// Calculate the distance between the robot and the target using the position of the target
double distance = Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2) + Math.pow(targetPosition.getZ(), 2));


//RANDOM OTHER SHIT GETTERS
public static double pipelineIndex = LimelightLib.getCurrentPipelineIndex(limelightName);
public static double captureLatency = LimelightLib.getLatency_Capture(limelightName);
public static double pipelineLatency = LimelightLib.getLatency_Pipeline(limelightName);



Vision() {
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}