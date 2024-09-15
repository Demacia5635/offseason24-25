package frc.robot.subsystems.chassis.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    

    
    

    /**
     * Constructor we use
     * 
     * @param moduleTranslationsMeters
     */
    public SwerveKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
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

    @Override
    public Twist2d toTwist2d(SwerveDriveWheelPositions start, SwerveDriveWheelPositions end) {
        if (start.positions.length != end.positions.length) {
            throw new IllegalArgumentException("Inconsistent number of modules!");
        }
        

        SwerveModulePosition[] newPositions = new SwerveModulePosition[start.positions.length];
        for (int i = 0; i < start.positions.length; i++) {
            var startModule = start.positions[i];
            var endModule = end.positions[i];
            newPositions[i] =
            new SwerveModulePosition(
                endModule.distanceMeters - startModule.distanceMeters,
                endModule.angle.plus(startModule.angle).div(2));
        };

        return toTwist2d(newPositions);
    }


}
