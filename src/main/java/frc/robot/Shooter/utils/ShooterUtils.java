package frc.robot.Shooter.utils;

import static frc.robot.Shooter.ShooterConstants.*;

/**
 * ShooterUtils
 */
public class ShooterUtils {

  static double lastVel = 0;

  public static double angleToDistance(double angle) {
    // return (2 * ANGLE_CHANGING_VAR.B * Math.cos(angle)
    //     + Math.sqrt(4 * Math.pow(ANGLE_CHANGING_VAR.B, 2) * Math.pow(Math.cos(angle), 2)
    //         - 4 * (Math.pow(ANGLE_CHANGING_VAR.B, 2) - Math.pow(ANGLE_CHANGING_VAR.A, 2))) / 2)
    //     - ANGLE_CHANGING_VAR.C;
    // return 2*ANGLE_CHANGING_VAR.B*Math.cos(Math.toRadians(angle)) - ANGLE_CHANGING_VAR.C; 
    /* if A = B */

    return ((ANGLE_CHANGING_VAR.B * Math.sin(Math.toRadians(180 - 2*angle)))/ (Math.sin(Math.toRadians(angle)))) - ANGLE_CHANGING_VAR.C;
  }

  public static double distanceToAngle(double distance) {
    // return Math.acos(
    //     ((Math.pow(ANGLE_CHANGING_VAR.A, 2) - Math.pow(ANGLE_CHANGING_VAR.B, 2)) / (ANGLE_CHANGING_VAR.C + distance)
    //         - ANGLE_CHANGING_VAR.C - distance) / (-2 * ANGLE_CHANGING_VAR.B));
    return Math.toDegrees(Math.acos((distance + ANGLE_CHANGING_VAR.C)/(2*ANGLE_CHANGING_VAR.B))); 
    /* if A = B */
  }

  public static double getUpMotorFF(double vel) {
    double ff = SHOOTER_PID_FF.UP_MOTOR_KS * Math.signum(vel) +
        SHOOTER_PID_FF.UP_MOTOR_KV * vel +
        SHOOTER_PID_FF.UP_MOTOR_KV2 * Math.pow(vel, 2) +
        SHOOTER_PID_FF.UP_MOTOR_KA * (vel - lastVel);

    lastVel = vel;
    return ff;
  }

  public static double getDownMotorFF(double vel) {
    double ff = SHOOTER_PID_FF.DOWN_MOTOR_KS * Math.signum(vel) +
        SHOOTER_PID_FF.DOWN_MOTOR_KV * vel +
        SHOOTER_PID_FF.DOWN_MOTOR_KV2 * Math.pow(vel, 2) +
        SHOOTER_PID_FF.DOWN_MOTOR_KA * (vel - lastVel);

    lastVel = vel;
    return ff;
  }
}