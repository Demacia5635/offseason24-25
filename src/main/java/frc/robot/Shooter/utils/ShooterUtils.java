package frc.robot.Shooter.utils;

import static frc.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * ShooterUtils
 */
public class ShooterUtils {
    public static double angleToDistance(double angle){
        return (2*B*Math.cos(angle)+Math.sqrt(4*Math.pow(B, 2)*Math.pow(Math.cos(angle), 2) - 4*(Math.pow(B, 2) - Math.pow(A, 2)))/2) - C;
        //return 2*B*Math.cos(angle) + C; if A = B
    }
    public static double distanceToAngle(double distance){
        return Math.acos(((Math.pow(A, 2)-Math.pow(B, 2))/(C+distance)-C-distance)/(-2*B));
        //return  Math.acos((distance + C)/(2*B)); if A = B
    }
}