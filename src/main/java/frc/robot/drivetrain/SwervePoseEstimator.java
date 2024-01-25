    package frc.robot.drivetrain;

    import com.kauailabs.navx.frc.AHRS;
    import com.revrobotics.CANSparkMax;
    import com.revrobotics.CANSparkMaxLowLevel;
    import com.revrobotics.RelativeEncoder;

    import frc.robot.utils.Constants;
    import frc.robot.utils.Constants.DriveConstants;

    public class SwervePoseEstimator {

        // Constants
        double wheelbaseRadius = DriveConstants.wheelBase;  //WHEEL DIAMETER

        //SPARKMAXES
        private CANSparkMax frontLeftMotor;
        private CANSparkMax frontRightMotor;
        private CANSparkMax rearLeftMotor;
        private CANSparkMax rearRightMotor;

        //NEO INTEGRATED ENCODERS
        private RelativeEncoder frontLeftEncoder;
        private RelativeEncoder frontRightEncoder;
        private RelativeEncoder rearLeftEncoder;
        private RelativeEncoder rearRightEncoder;

        //NAVX IMU
        private AHRS imu;

        //ROBOT POSE REPRESENTED IN THREE VARIABLES
        private double robotX = 0.0;
        private double robotY = 0.0;
        private double robotTheta = 0.0;

        // Absolute encoder positions for each module
        private double lastAbsoluteFrontLeft = 0.0;
        private double lastAbsoluteFrontRight = 0.0;
        private double lastAbsoluteRearLeft = 0.0;
        private double lastAbsoluteRearRight = 0.0;

        // Counts per revolution for absolute encoders
        private static final int absoluteEncoderCountsPerRevolution = 4096;  // Adjust based on your encoder specifications

        public SwervePoseEstimator() {

            //INITIALIZE SPARKMAX CONTROLLERS AND MOTORS
            frontLeftMotor = new CANSparkMax(Constants.DriveConstants.frontLeftDrivingCanId, CANSparkMaxLowLevel.MotorType.kBrushless);  // Replace with your IDs and motor types
            frontRightMotor = new CANSparkMax(Constants.DriveConstants.frontRightDrivingCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
            rearLeftMotor = new CANSparkMax(Constants.DriveConstants.rearLeftDrivingCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
            rearRightMotor = new CANSparkMax(Constants.DriveConstants.rearRightDrivingCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

            //INITIALIZE NEO INTEGRATED ENCODERS
            frontLeftEncoder = frontLeftMotor.getEncoder();
            frontRightEncoder = frontRightMotor.getEncoder();
            rearLeftEncoder = rearLeftMotor.getEncoder();
            rearRightEncoder = rearRightMotor.getEncoder();

            //INITIALIZE IMU 
            imu = DriveSubsystem.gyro;

            // Initialize absolute encoder positions
            lastAbsoluteFrontLeft = frontLeftEncoder.getPosition();
            lastAbsoluteFrontRight = frontRightEncoder.getPosition();
            lastAbsoluteRearLeft = rearLeftEncoder.getPosition();
            lastAbsoluteRearRight = rearRightEncoder.getPosition();
        }

        public void updatePose() {
            // Read encoder counts for each swerve module
            double encoderCountsFrontLeft = frontLeftEncoder.getPosition();
            double encoderCountsFrontRight = frontRightEncoder.getPosition();
            double encoderCountsRearLeft = rearLeftEncoder.getPosition();
            double encoderCountsRearRight = rearRightEncoder.getPosition();
        
            // Calculate distances using the new calculateDistance() method
            double distanceFrontLeft = calculateDistance(encoderCountsFrontLeft, lastAbsoluteFrontLeft);
            double distanceFrontRight = calculateDistance(encoderCountsFrontRight, lastAbsoluteFrontRight);
            double distanceRearLeft = calculateDistance(encoderCountsRearLeft, lastAbsoluteRearLeft);
            double distanceRearRight = calculateDistance(encoderCountsRearRight, lastAbsoluteRearRight);
        
            // Calculate robot's translational and rotational distances
            double deltaTransX = (distanceFrontLeft + distanceFrontRight + distanceRearLeft + distanceRearRight) / 4.0;
            double deltaTransY = (distanceFrontLeft - distanceFrontRight - distanceRearLeft + distanceRearRight) / 4.0;  // Sideways movement
            double deltaRotZ = (distanceFrontRight - distanceFrontLeft - distanceRearRight + distanceRearLeft) / (4.0 * wheelbaseRadius);
        
            // Adjust the movement directions based on the orientation
            double frontLeftOrientation = frontLeftEncoder.getPosition() * 360.0 / absoluteEncoderCountsPerRevolution; // Convert to degrees
            deltaTransX = deltaTransX * Math.cos(Math.toRadians(frontLeftOrientation)) - deltaTransY * Math.sin(Math.toRadians(frontLeftOrientation));
            deltaTransY = deltaTransX * Math.sin(Math.toRadians(frontLeftOrientation)) + deltaTransY * Math.cos(Math.toRadians(frontLeftOrientation));
        
            // Integrate IMU heading to obtain robot's heading
            double imuHeading = imu.getYaw();
            double integratedHeading = imuHeading + deltaRotZ;
        
            // Convert the combined translational distances to X and Y components
            double deltaX = deltaTransX * Math.cos(robotTheta) - deltaTransY * Math.sin(robotTheta);
            double deltaY = deltaTransX * Math.sin(robotTheta) + deltaTransY * Math.cos(robotTheta);
        
            // Update robot's pose
            updateRobotPose(deltaX, deltaY, integratedHeading);
        
            // Update the last absolute encoder positions
            lastAbsoluteFrontLeft = encoderCountsFrontLeft;
            lastAbsoluteFrontRight = encoderCountsFrontRight;
            lastAbsoluteRearLeft = encoderCountsRearLeft;
            lastAbsoluteRearRight = encoderCountsRearRight;
        }
        
        
        // New method to calculate distance based on absolute encoder positions
        private double calculateDistance(double currentAbsolute, double lastAbsolute) {
            // Calculate the change in absolute angle
            double deltaAbsolute = currentAbsolute - lastAbsolute;
        
            // Convert the change in absolute angle to distance (arc length)
            return (2 * Math.PI * wheelbaseRadius * deltaAbsolute) / absoluteEncoderCountsPerRevolution;
        }
        


        private void updateRobotPose(double x, double y, double theta) {
            // Implement your method to update robot pose (e.g., send to NetworkTables)
            robotX += x;
            robotY += y;
            robotTheta = theta;
        }

        // Methods for retrieving pose information
        public double getRobotX() {
            return robotX;
        }

        public double getRobotY() {
            return robotY;
        }

        public double getRobotTheta() {
            return robotTheta;
        }

        // New methods to get current position and individual values
        public double[] getCurrentPosition() {
            return new double[]{robotX, robotY, robotTheta};
        }

        public double getCurrentX() {
            return robotX;
        }

        public double getCurrentY() {
            return robotY;  
        }

        public double getCurrentTheta() {
            return robotTheta;
        }
    }
