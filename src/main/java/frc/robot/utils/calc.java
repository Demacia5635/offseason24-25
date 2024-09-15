package frc.robot.utils;

import frc.robot.Constants;

public class calc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;
    private double id;
    private double sumdegry;

    public calc(double height, double tx, double ty, double x_offset, double y_offset, double id) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        this.height = height;
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
}