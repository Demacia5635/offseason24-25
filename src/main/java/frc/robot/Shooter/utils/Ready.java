// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.utils;

import static frc.robot.Shooter.ShooterConstants.*;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class Ready {
    public static boolean isAngleReady(double wantedAngle){
        return Math.abs(wantedAngle - RobotContainer.angleChanging.getAngle()) <= ANGLE_ZONE;
    }
    public static boolean isUpMotorReady(double wantedUpMotorVel){
        return Math.abs(wantedUpMotorVel - RobotContainer.shooter.getUpMotorVel()) <= UP_MOTOR_VEL_ZONE;
    }
    public static boolean isDownMotorReady(double wantedDownMotorVel){
        return Math.abs(wantedDownMotorVel - RobotContainer.shooter.getDownMotorVel()) <= DOWN_MOTOR_VEL_ZONE;
    }
}
