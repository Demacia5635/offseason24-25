package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class pose {
    private String objects;
    private double dists;
    private double angles;
    private Pose2d robotPose;
    private double robotYaw;

    public pose(String objects, double dists, double angles, double robotYaw) {
        this.objects = objects;
        this.dists = dists;
        this.angles = angles;
        this.robotYaw = robotYaw;
    }

    // Calculate the robot's pose based on detected objects
    public Pose2d calcMyPose() {
        Translation2d obj = Constants.dic.get(objects);
        if (obj != null) {
            Pose2d point = calculatePoint(obj, dists, angles);
            robotPose = average(robotPose, point);
        }
        return robotPose;
    }

    // Calculate a point based on object position, distance, and angle
    private Pose2d calculatePoint(Translation2d obj, double dist, double angle) {
        double globalAngle = (robotYaw + angle) % 360;
        Translation2d relativePosition = new Translation2d(dist, Rotation2d.fromDegrees(globalAngle));
        Translation2d globalPosition = obj.minus(relativePosition);
        return new Pose2d(globalPosition, Rotation2d.fromDegrees(robotYaw));
    }

    // Calculate average of two poses
    public Pose2d average(Pose2d p1, Pose2d p2) {
        if (p1 == null) return p2;
        if (p2 == null) return p1;
        Translation2d avgTranslation = p1.getTranslation().plus(p2.getTranslation()).div(2);
        Rotation2d avgRotation = p1.getRotation().plus(p2.getRotation()).times(0.5);
        return new Pose2d(avgTranslation, avgRotation);
    }
}