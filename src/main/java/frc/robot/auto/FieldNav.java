package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.Constants.FieldConstants;

public class FieldNav {

    //DEFINE FIELD DIMENSIONS
    public static final double field_length = FieldConstants.field_length;
    public static final double field_width = FieldConstants.field_width;

    //DEFINE ORIGIN POSITION
    private static final double origin_x = FieldConstants.origin_x;
    private static final double origin_y = FieldConstants.origin_y;

    //DEFINE FORBIDDEN ZONES
    public static final List<RectangularZone> forbiddenZones = new ArrayList<>();

    static {
        //EXAMPLE FORBIDDEN ZONE
        forbiddenZones.add(new RectangularZone(5.0, 10.0, 15.0, 20.0));
    }

    //CONVERT FIELD COORDINATES TO POSE2D
public static Pose2d convertToPose2d(double x, double y) {
    //CHECK IF ITS INSIDE THE FIELD
    if (x >= 0 && x <= field_length && y >= 0 && y <= field_width) {
        //CHECK IF ITS IN A FORBIDDEN ZONE
        for (RectangularZone zone : forbiddenZones) {
            if (zone.contains(x, y)) {
                System.out.println("Error: Coordinates are in a forbidden zone");
                return null;
            }
        }
        //CONVERT TO POSE2D
        return new Pose2d(new Translation2d(x - origin_x, y - origin_y), new Rotation2d()); //ASSUMING ZERO ROTATION
    } else {
        //COORDINATES ARE OUTSIDE OF FIELD BOUNDARIES
        System.out.println("Error: Coordinates are outside the field boundaries");
        return null;
    }
}



    //HELPER CLASS TO REPRESENT RECTANGULAR ZONES
    private static class RectangularZone {
        public final double minX;
        public final double minY;
        public final double maxX;
        public final double maxY;

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
