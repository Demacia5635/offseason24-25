package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class pose {
    private String objects;
    private double dists;
    private double angles;
    private Translation2d pose;

    public pose(String objects, double dists, double angles) {
        this.objects = objects;
        this.dists = dists;
        this.angles = angles;
    }

    // Calculate the robot's pose based on detected objects
    public Translation2d calcMyPose() {

        Translation2d obj = Constants.dic.get(objects);
        if (obj != null) {
            Translation2d point = calculatePoint(obj, dists, angles);
            pose = average(pose, point);
        }

        return pose;
    }

    // Calculate a point based on object position, distance, and angle
    private Translation2d calculatePoint(Translation2d obj, double dist, double angle) {
        Translation2d trigo = trigo(dist, angle % 90);
        double x = obj.getX() + (angle > 90 && angle <= 270 ? trigo.getX() : -trigo.getX());
        double y = obj.getY() + (angle > 180 ? trigo.getY() : -trigo.getY());
        return new Translation2d(x, y);
    }

    // Calculate trigonometric components
    private Translation2d trigo(double dist, double angle) {
        double radians = Math.toRadians(angle);
        double x = dist * Math.cos(radians);
        double y = dist * Math.sin(radians);
        return new Translation2d(x, y);
    }

    // Calculate average of two points
    public Translation2d average(Translation2d p1, Translation2d p2) {
        if (p1 == null) return p2;
        if (p2 == null) return p1;
        return new Translation2d((p1.getX() + p2.getX()) / 2, (p1.getY() + p2.getY()) / 2);
    }
}