package frc.robot.arm;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.Constants.DriveConstants;

public class Arm {

    public static CANSparkMax leftArm = DriveConstants.leftArm;  
    public static CANSparkMax rightArm = DriveConstants.rightArm;
    public static AbsoluteEncoder armEncoder = rightArm.getAbsoluteEncoder(Type.kDutyCycle);
    public static double armEncoderReading = armEncoder.getPosition();
    double aP = 0;
    double aI = 0;
    double aD = 0;
    PIDController armPID = new PIDController(aP, aI, aD);
    PIDConstants armPIDConstants = new PIDConstants(aP, aI, aD);

    public void rotateToDegrees(double degree) {
        double armSetpoint = degree;
        double turnValue = armPID.calculate(armEncoderReading, armSetpoint);
        rotateVector(turnValue);
    }

    public static void rotateVector(double speed) {
      leftArm.set(speed);
      rightArm.set(-speed);
    }


    public void armDontMove() {
        double armSetpoint = armEncoderReading; //SET THE TARGET POSITION TO THE CURRENT POSITION
        double turnValue = armPID.calculate(armEncoderReading, armSetpoint); //SET THE PID CONTROLLER TO KEEPING THE ARM IN PLACE
        rotateVector(turnValue);
    }
    
}
