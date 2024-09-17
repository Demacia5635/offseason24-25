package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class TagPoseCalc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;
    private double id;
    private double sumdegry;
    private Pose2d point;
    private double robotYaw;


    public TagPoseCalc(double tx, double ty, double x_offset, double y_offset, double id, double robotYaw) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        this.height = Constants.HEIGHT_MAP.get(id);
        this.robotYaw = robotYaw;
    }

    // Calculate distance to the object
    public double GetDist() {
        sumdegry = ty + Constants.LimelightAngle;
        sumdegry = Math.toRadians(sumdegry);
        return ((Math.abs(height - Constants.LimelightHight)) / (Math.tan(sumdegry))) + x_offset;
    }

    // Get object identifier (note or AprilTag)
    public String GetObj() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    // Calculate angle to the object
    public double GetAngle() {
        double angle_rad = Math.toRadians(tx);
        double mol = (this.GetDist() - x_offset) * Math.tan(angle_rad);
        mol = Math.abs(y_offset - mol);
        angle_rad = Math.atan2(mol, this.GetDist());
        return Math.toDegrees(angle_rad);
    }

    public Pose2d calcMyPose() {
        Translation2d obj = Constants.dic.get(this.GetObj());
        if (obj != null) {
            point = calculatePoint(obj, this.GetDist(), this.GetAngle());
        }
        return point;
    }

    // Calculate a point based on object position, distance, and angle
    private Pose2d calculatePoint(Translation2d obj, double dist, double angle) {
        double globalAngle = (robotYaw + angle) % 360;
        Translation2d relativePosition = new Translation2d(dist, Rotation2d.fromDegrees(globalAngle));
        Translation2d globalPosition = obj.minus(relativePosition);
        return new Pose2d(globalPosition, Rotation2d.fromDegrees(robotYaw));
    }
}