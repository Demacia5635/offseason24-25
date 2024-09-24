package frc.robot.subsystems.chassis.Kinematics;

import org.opencv.core.Scalar;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

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
    public double MINalpha=5;
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
        return vel * Constants.CYCLE_DT;
    }

    public SwerveModuleState straightPath(Translation2d moduleLocationDifference,SwerveModuleState state)   {
        SwerveModuleState newState = new SwerveModuleState(moduleLocationDifference.getNorm()/0.02,state.angle);
        return newState;
    }

     public SwerveModuleState curvedPath(Rotation2d alpha,Translation2d moduleLocationDifference,Translation2d moduleEstimatedPos,SwerveModuleState prevState,Pose2d curPose,Pose2d estimatedPose){
        double radius = moduleLocationDifference.getNorm() * Math.sin((Math.PI / 2 ) - alpha.getRadians()) / Math.sin(alpha.getRadians() * 2);

        double moduleV = alpha.times(2 * radius).getRadians() / Constants.CYCLE_DT;    //(2alpha * d * sin(0.5pi - alpha)/sin(2alpha))/0.02 = Vn
        
        double startingModuleRadians = prevState.angle.getRadians();
        double chassisDiffRadians = estimatedPose.getRotation().minus(curPose.getRotation()).getRadians();
        
        double moduleAngle = startingModuleRadians+(2*alpha.getRadians())-chassisDiffRadians;// d0 + 2alpha - delta(Beta) = Dn
        return new SwerveModuleState(moduleV,new Rotation2d(moduleAngle));
     }

    /**
     * Rotate the speeds counter to omega - to drive stright
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, Pose2d curPose, SwerveModuleState[] prevStates) {
        Pose2d estimatedPose = new Pose2d(curPose.getX() + getCycleDistance(speeds.vxMetersPerSecond), curPose.getY() + getCycleDistance(speeds.vyMetersPerSecond),
            curPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * 0.02))); //the estimated pose2d of the chassis location and direction
        SwerveModuleState[] newModuleStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            Translation2d moduleEstimatedPos = estimatedPose.getTranslation().plus(moduleTranslationsMeters[i].rotateBy(estimatedPose.getRotation().minus(curPose.getRotation())));//the module estimated pos
            Translation2d moduleLocationDifference = moduleEstimatedPos.minus(curPose.getTranslation().plus(moduleTranslationsMeters[i].rotateBy(curPose.getRotation()))); //the delta x of the module between previous and estimate
            Rotation2d alpha = moduleLocationDifference.getAngle().minus(prevStates[i].angle); //finding alpha
            newModuleStates[i] = alpha.getDegrees()>=MINalpha ? curvedPath(alpha,moduleLocationDifference,moduleEstimatedPos,prevStates[i],curPose,estimatedPose) : straightPath(moduleLocationDifference,prevStates[i]);
        }
        return newModuleStates;
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
