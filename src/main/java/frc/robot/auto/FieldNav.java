package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldNav {

    //DEFINE FIELD DIMENSIONS
    private static final double field_length = 54.0; //FEET
    private static final double field_width = 27.0; //FEET

    //DEFINE ORIGIN POSITION
    private static final double ORIGIN_X = 0.0;
    private static final double ORIGIN_Y = 0.0;

    //DEFINE FORBIDDEN ZONES
    private static final List<RectangularZone> forbiddenZones = new ArrayList<>();

    static {
        //EXAMPLE FORBIDDEN ZONE
        forbiddenZones.add(new RectangularZone(5.0, 10.0, 15.0, 20.0));
    }

    //CONVERT FIELD COORDINATES TO POSE2D
    public static Pose2d convertToPose2d(double x, double y) {
        //CHECK IF ITS IN THE FIELD BOUNDARIES
        if (x >= 0 && x <= field_length && y >= 0 && y <= field_width) {
            //CHECK IF THEY ARE IN A FORBIDDEN ZONE
            for (RectangularZone zone : forbiddenZones) {
                if (zone.contains(x, y)) {
                    System.out.println("Error: Coordinates are in a forbidden zone");
                    return null;
                }
            }
            //CONVERT TO POSE2D
            return new Pose2d(new Translation2d(x, y), new Rotation2d()); //ASSUMING ZERO ROTATION
        } else {
            //COORDINATES OUTSIDE OF BOUNDARIES
            System.out.println("Error: Coordinates are outside the field boundaries");
            return null;
        }
    }


    //HELPER CLASS TO REPRESENT RECTANGULAR ZONES
    private static class RectangularZone {
        private final double minX;
        private final double minY;
        private final double maxX;
        private final double maxY;

        public RectangularZone(double minX, double minY, double maxX, double maxY) {
            this.minX = minX;
            this.minY = minY;
            this.maxX = maxX;
            this.maxY = maxY;
        }

        public boolean contains(double x, double y) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }
}
