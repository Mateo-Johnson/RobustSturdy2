package frc.robot.vision.commands;

import org.opencv.core.Point;

import frc.robot.turret.ShooterSubsystem;
import frc.robot.vision.Vision;

public class AutoTurretShoot {
    
    private ShooterSubsystem shooter;

    Point detectTarget() {
        //CHECK IF THE LIMELIGHT HAS A TARGET
        if (Vision.tV == true ) {
            // Get target coordinates (x, y) in screen space
            double targetX = Vision.tX;
            double targetY = Vision.tY;

            // Return target coordinates
            return new Point(targetX, targetY);
        } else {
            // No target detected, return null
            return null;
        }
    }
    
}
