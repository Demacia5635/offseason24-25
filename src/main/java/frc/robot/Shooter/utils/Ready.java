// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.utils;

import frc.robot.RobotContainer;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.MAX_ERRORS;
import frc.robot.Shooter.Commands.GoToAngle;

/** Add your docs here. */
public class Ready {
    public static boolean isAngleReady(double wantedAngle){
        return Math.abs(wantedAngle - RobotContainer.angleChanging.getAngle()) <= MAX_ERRORS.ANGLE_MAX_ERRORS;
    }
    public static boolean isUpMotorReady(double wantedUpMotorVel){
        return Math.abs(wantedUpMotorVel - RobotContainer.shooter.getUpMotorVel()) <= MAX_ERRORS.UP_MOTOR_VEL_MAX_ERRORS;
    }
    public static boolean isDownMotorReady(double wantedDownMotorVel){
        return Math.abs(wantedDownMotorVel - RobotContainer.shooter.getDownMotorVel()) <= MAX_ERRORS.DOWN_MOTOR_VEL_MAX_ERRORS;
    }
    public static boolean isGoodState(STATE state){
        return (state == STATE.SPEAKER || state == STATE.AMP || state == STATE.STAGE || state == STATE.SUBWOFFER || state == STATE.DELIVERY_MID || state == STATE.DELIVERY_RIVAL);
    }
    public static boolean isSeeAprilTag(){
        return true;
    }
    public static boolean isNearAmp(){
        return true;
    }
    public static boolean isReady(double upMotorVel, double downMotorVel, STATE state){
        return GoToAngle.isAngleReady
          && isUpMotorReady(upMotorVel)
          && isUpMotorReady(downMotorVel)
          && isGoodState(state)
          && ((state == STATE.AMP && isNearAmp()) 
            || state != STATE.AMP)
          && (((state == STATE.SPEAKER) && isSeeAprilTag()) 
            || state != STATE.SPEAKER);
    }
}
