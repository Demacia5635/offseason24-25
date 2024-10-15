package frc.robot.Shooter.utils;

import static frc.robot.Shooter.ShooterConstants.*;

/**
 * ShooterUtils
 */
public class ShooterUtils {
    public static double angleToDistance(double angle){
        return (2*ANGLE_CHANGING_VAR.B*Math.cos(angle)+Math.sqrt(4*Math.pow(ANGLE_CHANGING_VAR.B, 2)*Math.pow(Math.cos(angle), 2) - 4*(Math.pow(ANGLE_CHANGING_VAR.B, 2) - Math.pow(ANGLE_CHANGING_VAR.A, 2)))/2) - ANGLE_CHANGING_VAR.C;
        //return 2*B*Math.cos(angle) + C; if A = B
    }
    public static double distanceToAngle(double distance){
        return Math.acos(((Math.pow(ANGLE_CHANGING_VAR.A, 2)-Math.pow(ANGLE_CHANGING_VAR.B, 2))/(ANGLE_CHANGING_VAR.C+distance)-ANGLE_CHANGING_VAR.C-distance)/(-2*ANGLE_CHANGING_VAR.B));
        //return  Math.acos((distance + C)/(2*B)); if A = B
    }

    public static double getUpMotorFF(double vel) {
        return SHOOTER_PID_FF.UP_MOTOR_KS * Math.signum(vel) + 
        SHOOTER_PID_FF.UP_MOTOR_KV * vel + 
        SHOOTER_PID_FF.UP_MOTOR_KV2 * Math.pow(vel, 2);
    } 

    public static double getDownMotorFF(double vel) {
        return SHOOTER_PID_FF.UP_MOTOR_KS * Math.signum(vel) + 
        SHOOTER_PID_FF.UP_MOTOR_KV * vel + 
        SHOOTER_PID_FF.UP_MOTOR_KV2 * Math.pow(vel, 2);
    }
}