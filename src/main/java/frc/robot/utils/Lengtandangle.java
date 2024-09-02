// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import java.util.HashMap;
/** Add your docs here. */
public class Lengtandangle {

    double tx;
    double ty;
    double length;
    double hightO;
    double hight;
    int id;
    public Lengtandangle(int id){
        this.id = id;
        this.hightO = Constants.HEIGHT_MAP.get(id);
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
    }
    public double findLength(){

        return (Math.abs(hightO-Constants.LimelightHight))/ Math.tan(ty);
    }
    public double findAngle(){
        return tx;
    }
    
}
