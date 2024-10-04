// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class KinematicsFix {

    public static final double VX_VY_CHANGE_RATIO = -0.1;
    public static final double MIN_RATIO_CHANGE = Math.toRadians(20);
    public static final double MIN_OMEGA_TO_CHANGE = 0.1;
    public static final double MAX_OMEGA_TO_CHANGE = 1;
    public static final double MIN_OMEGA_CHANGE_AMOUNT = 1.2;
    public static final double MAX_OMEGA_CHANGE_AMOUNT = 1;

    private static final double _A = (MAX_OMEGA_CHANGE_AMOUNT-MIN_OMEGA_CHANGE_AMOUNT)/(MAX_OMEGA_TO_CHANGE-MIN_OMEGA_TO_CHANGE);
    private static final double _B = MAX_OMEGA_CHANGE_AMOUNT - MAX_OMEGA_TO_CHANGE*_A;

    private static double percentCorrection = 0.8;
    private static double fixOmega(double omega) {
        if(Math.abs(omega) > MIN_OMEGA_TO_CHANGE && Math.abs(omega) < MAX_OMEGA_TO_CHANGE) {
            return (omega * _A + _B)*Math.signum(omega);
        }
        return omega;
    }

    public static ChassisSpeeds fixChassisSpeeds(ChassisSpeeds speeds){
        speeds = fixVelWhileRotate(speeds);
        speeds.omegaRadiansPerSecond = fixOmega(speeds.omegaRadiansPerSecond);

        double ratio = speeds.omegaRadiansPerSecond > MIN_RATIO_CHANGE ? VX_VY_CHANGE_RATIO : 0;
        ChassisSpeeds s = speeds;
        if(ratio != 0) {
            Translation2d newV = new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond).rotateBy(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * ratio));
            s = new ChassisSpeeds(newV.getX(), newV.getY(), speeds.omegaRadiansPerSecond);
        }
        return s;
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
