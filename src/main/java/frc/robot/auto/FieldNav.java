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
    public static final List<ForbiddenZone> forbiddenZones = new ArrayList<>();

    static {
        //EXAMPLE FORBIDDEN ZONES
        forbiddenZones.add(new RectangularZone(5.0, 10.0, 15.0, 20.0));
        forbiddenZones.add(new CircularZone(25.0, 15.0, 5.0));
        forbiddenZones.add(new TriangularZone(35.0, 10.0, 40.0, 20.0, 37.5, 15.0));
        forbiddenZones.add(new HexagonalZone(50.0, 10.0, 60.0, 10.0, 65.0, 15.0, 60.0, 20.0, 50.0, 20.0, 45.0, 15.0));
        forbiddenZones.add(new EllipseZone(75.0, 15.0, 5.0, 10.0));
        forbiddenZones.add(new IrregularPolygonZone(new Translation2d(85.0, 10.0), new Translation2d(87.5, 12.5), new Translation2d(90.0, 10.0), new Translation2d(87.5, 7.5)
        ));
    }

    //CONVERT FIELD COORDINATES TO POSE2D
    public static Pose2d convertToPose2d(double x, double y) {
        //CHECK IF ITS INSIDE THE FIELD
        if (x >= 0 && x <= field_length && y >= 0 && y <= field_width) {
            //CHECK IF ITS IN A FORBIDDEN ZONE
            for (ForbiddenZone zone : forbiddenZones) {
                if (zone.contains(x, y)) {
                    System.out.println("ERROR: COORDINATES IN THE FORBIDDEN ZONE");
                    return null;
                }
            }
            //CONVERT TO POSE2D
            return new Pose2d(new Translation2d(x - origin_x, y - origin_y), new Rotation2d()); //ASSUMING ZERO ROTATION
        } else {
            //COORDINATES ARE OUTSIDE OF FIELD BOUNDARIES
            System.out.println("ERROR: COORDINATES ARE OUTSIDE THE FIELD BOUNDARIES");
            return null;
        }
    }

    //HELPER INTERFACE TO REPRESENT FORBIDDEN ZONES
    private interface ForbiddenZone {
        boolean contains(double x, double y);
    }

    //RECTANGULAR FORBIDDEN ZONE IMPLEMENTATION
    private static class RectangularZone implements ForbiddenZone {
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

        @Override
        public boolean contains(double x, double y) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }

    //CIRCULAR FORBIDDEN ZONE IMPLEMENTATION
    private static class CircularZone implements ForbiddenZone {
        private final double centerX;
        private final double centerY;
        private final double radius;

        public CircularZone(double centerX, double centerY, double radius) {
            this.centerX = centerX;
            this.centerY = centerY;
            this.radius = radius;
        }

        @Override
        public boolean contains(double x, double y) {
            return Math.sqrt(Math.pow(x - centerX, 2) + Math.pow(y - centerY, 2)) <= radius;
        }
    }

    //TRIANGULAR FORBIDDEN ZONE IMPLEMENTATION
    private static class TriangularZone implements ForbiddenZone {
        private final double x1;
        private final double y1;
        private final double x2;
        private final double y2;
        private final double x3;
        private final double y3;

        public TriangularZone(double x1, double y1, double x2, double y2, double x3, double y3) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
            this.x3 = x3;
            this.y3 = y3;
        }

        @Override
        public boolean contains(double x, double y) {
            //IMPLEMENT POINT-IN-TRIANGLE CHECK
            //YOU CAN USE USE BARYCENTRIC COORDINATES TOO
            //BOUNDING BOX CHECK
            double minX = Math.min(Math.min(x1, x2), x3);
            double minY = Math.min(Math.min(y1, y2), y3);
            double maxX = Math.max(Math.max(x1, x2), x3);
            double maxY = Math.max(Math.max(y1, y2), y3);
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }

    //HEXAGONAL FORBIDDEN ZONE IMPLEMENTATION
    private static class HexagonalZone implements ForbiddenZone {
        private final List<Translation2d> vertices;

        public HexagonalZone(double... coordinates) {
            vertices = new ArrayList<>();
            for (int i = 0; i < coordinates.length; i += 2) {
                vertices.add(new Translation2d(coordinates[i], coordinates[i + 1]));
            }
        }

        @Override
        public boolean contains(double x, double y) {
            //IMPLEMENT POINT-IN-POLYGON CHECK
            //YOU CAN USE RAYCASTING OR OTHER METHODS
            //ANOTHER BOUNDING BOX CHECK
            double minX = vertices.stream().mapToDouble(v -> v.getX()).min().orElse(0);
            double minY = vertices.stream().mapToDouble(v -> v.getY()).min().orElse(0);
            double maxX = vertices.stream().mapToDouble(v -> v.getX()).max().orElse(0);
            double maxY = vertices.stream().mapToDouble(v -> v.getY()).max().orElse(0);
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }

    //ELLIPSE FORBIDDEN ZONE IMPLEMENTATION
    private static class EllipseZone implements ForbiddenZone {
        private final double centerX;
        private final double centerY;
        private final double semiMajorAxis;
        private final double semiMinorAxis;

        public EllipseZone(double centerX, double centerY, double semiMajorAxis, double semiMinorAxis) {
            this.centerX = centerX;
            this.centerY = centerY;
            this.semiMajorAxis = semiMajorAxis;
            this.semiMinorAxis = semiMinorAxis;
        }

        @Override
        public boolean contains(double x, double y) {
            //IMPLEMENT POINT IN-ELLIPSE CHECK
            //FOR SIMPLICITY THIS USES THE ELLIPSE FORMULA 
            return Math.pow((x - centerX) / semiMajorAxis, 2) + Math.pow((y - centerY) / semiMinorAxis, 2) <= 1.0;
        }
    }

    //IRREGULAR POLYGON FORBIDDEN ZONE IMPLEMENTATION
    private static class IrregularPolygonZone implements ForbiddenZone {
        private final List<Translation2d> vertices;

        public IrregularPolygonZone(Translation2d... vertices) {
            this.vertices = List.of(vertices);
        }

        @Override
        public boolean contains(double x, double y) {
            //IMPLEMENT POINT-IN-POLYGON CHECK 
            //YOU COULD USE RAY-CASTING
            //BOUNDING BOX CHECK
            double minX = vertices.stream().mapToDouble(v -> v.getX()).min().orElse(0);
            double minY = vertices.stream().mapToDouble(v -> v.getY()).min().orElse(0);
            double maxX = vertices.stream().mapToDouble(v -> v.getX()).max().orElse(0);
            double maxY = vertices.stream().mapToDouble(v -> v.getY()).max().orElse(0);
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }
}
