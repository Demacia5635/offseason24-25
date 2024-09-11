package frc.robot.utils;

import frc.robot.Constants;

public class calc {
    private double height;
    private double id;
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;

    public calc(double id, double tx, double ty) {
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        height = Constants.HEIGHT_MAP.get(id);
    }

    // Calculate distance to the object
    public double GetDist() {
        return (Math.abs(height - Constants.LimelightHight) * Math.tan(Math.toRadians(ty + Constants.LimelightAngle))) + x_offset;
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