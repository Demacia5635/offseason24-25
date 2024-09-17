package frc.robot.subsystems.chassis.Kinematics;

import edu.wpi.first.math.geometry.Pose2d;
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
    

   

    public SwerveModuleState[] states;
    Translation2d[] moduleTranslationsMeters;
    /**
     * Constructor we use
     * 
     * @param moduleTranslationsMeters
     */
    public SwerveKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
        this.moduleTranslationsMeters = moduleTranslationsMeters;
        
    }

    private double getCycleDistance(double vel){
        return vel * 0.02;
    }
   
    /**
     * Rotate the speeds counter to omega - to drive stright
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, Pose2d curPose, SwerveModuleState[] prevStates) {
        Pose2d estimatedPose = new Pose2d(curPose.getX() + getCycleDistance(speeds.vxMetersPerSecond), curPose.getY() + getCycleDistance(speeds.vyMetersPerSecond),
            curPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * 0.02)));
        for(int i = 0; i < 4; i++){
            Translation2d modulePos = estimatedPose.getTranslation().plus(moduleTranslationsMeters[i].rotateBy(estimatedPose.getRotation()));
            Translation2d prevToEstimated = modulePos.minus(curPose.getTranslation().plus(moduleTranslationsMeters[i]));
            
        }
        return new SwerveModuleState[4];
    }

    /**
     * Rotate the speeds back to omega - to drive stright
     */
    @Override
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        return new ChassisSpeeds();
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
