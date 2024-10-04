// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class KinematicsFix {

    private static double rotationKp = 0.1;
    private static double percentCorrection = 0.8;
    public static ChassisSpeeds fixOmega(ChassisSpeeds speeds){
        speeds = fixVelWhileRotate(speeds);
        Translation2d velVector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        velVector.rotateBy(new Rotation2d(-speeds.omegaRadiansPerSecond * rotationKp));
        return new ChassisSpeeds(velVector.getX(), velVector.getY(), speeds.omegaRadiansPerSecond);
    }
    private static ChassisSpeeds fixVelWhileRotate(ChassisSpeeds speeds){
        if(Math.abs(speeds.omegaRadiansPerSecond) <= 0.1 
            || Math.abs(speeds.vxMetersPerSecond) < 4 
            || Math.abs(speeds.vyMetersPerSecond) < 4) return speeds;

        return new ChassisSpeeds(speeds.vxMetersPerSecond * percentCorrection,
            speeds.vyMetersPerSecond * percentCorrection,
            speeds.omegaRadiansPerSecond * percentCorrection);

        
    }
}
