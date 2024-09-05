// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
/** Add your docs here. */
public class Lengtandangle {


    private double hight;
    private int id;
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;
    public Lengtandangle(int id, double tx, double ty){
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        hight = Constants.HEIGHT_MAP.get(id);
        
    }
    public double GetDist(){

        return (Math.abs(hight-Constants.LimelightHight) * Math.tan(ty+Constants.LimelightAngle)) + x_offset;
    }
    public String GetObj(){
        
        return id == 0 ? "note" : "tag_" + id;
    }
    public double GetAngle(){
        double angle_rad = Math.toRadians(tx);
        double mol = (this.GetDist()-x_offset)*Math.tan(angle_rad);
        mol = Math.abs(y_offset - mol);
        angle_rad = Math.atan2(mol, this.GetDist());
        return Math.toDegrees(angle_rad);
    }

    
}
