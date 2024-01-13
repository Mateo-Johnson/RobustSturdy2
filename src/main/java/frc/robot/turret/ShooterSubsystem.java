package frc.robot.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private double currentAngle;

    public void setAngle(double angle) {
        // Implement code to set the angle of the shooter
        // This could involve controlling motors or other mechanisms
        // For demonstration purposes, let's just print the angle
        System.out.println("Setting shooter angle to: " + angle);
        currentAngle = angle;
    }

    public double getCurrentAngle() {
        // Implement code to get the current angle of the shooter
        // For demonstration purposes, let's return a constant value
        return currentAngle;
    }

    // Other methods and variables specific to your shooter subsystem...
}
