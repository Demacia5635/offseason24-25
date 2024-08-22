package frc.robot.chassis.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class to enhance WPILIB SwerveDriveKinematics
 * 
 * In sewerveToModuleState 
 *  - if omega is in low range - increase the rate
 *  - if omega is non zero - change the vx/vy to compensate for the turn
 * 
 * In toChassisSpeeds
 *   - do the reverse - if Omega is non zero - change the result vx/vy as compensated
 */
public class SwerveKinematics extends SwerveDriveKinematics {

    public static final double VX_VY_CHANGE_RATIO = -0.1;
    public static final double MIN_RATIO_CHANGE = Math.toRadians(20);
    public static final double MIN_OMEGA_TO_CHANGE = 0.1;
    public static final double MAX_OMEGA_TO_CHANGE = 1;
    public static final double MIN_OMEGA_CHANGE_AMOUNT = 1.2;
    public static final double MAX_OMEGA_CHANGE_AMOUNT = 1;

    private static final double _A = (MAX_OMEGA_CHANGE_AMOUNT-MIN_OMEGA_CHANGE_AMOUNT)/(MAX_OMEGA_TO_CHANGE-MIN_OMEGA_TO_CHANGE);
    private static final double _B = MAX_OMEGA_CHANGE_AMOUNT - MAX_OMEGA_TO_CHANGE*_A;

    /**
     * Constructor we use
     * 
     * @param moduleTranslationsMeters
     */
    public SwerveKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
    }

    public static double fixOmega(double omega) {
        if(Math.abs(omega) > MIN_OMEGA_TO_CHANGE && Math.abs(omega) < MAX_OMEGA_TO_CHANGE) {
            return (omega * _A + _B)*Math.signum(omega);
        }
        return omega;
    }


    /**
     * Rotate the speeds counter to omega - to drive stright
     */
    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds) {
        speeds.omegaRadiansPerSecond = fixOmega(speeds.omegaRadiansPerSecond);
        double ratio = speeds.omegaRadiansPerSecond > MIN_RATIO_CHANGE ? VX_VY_CHANGE_RATIO : 0;
        ChassisSpeeds s = speeds;
        if(ratio != 0) {
            Translation2d newV = new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond).rotateBy(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * ratio));
            s = new ChassisSpeeds(newV.getX(), newV.getY(), speeds.omegaRadiansPerSecond);
        }
        return super.toSwerveModuleStates(s);
    }

    /**
     * Rotate the speeds back to omega - to drive stright
     */
    @Override
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        ChassisSpeeds speeds = super.toChassisSpeeds(moduleStates);
        double ratio = speeds.omegaRadiansPerSecond > MIN_RATIO_CHANGE ? -VX_VY_CHANGE_RATIO : 0;
        ChassisSpeeds s = speeds;
        if(ratio != 0) {
            Translation2d newV = new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond).rotateBy(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * ratio));
            s = new ChassisSpeeds(newV.getX(), newV.getY(), speeds.omegaRadiansPerSecond);
        }
        return s;
    }
}
