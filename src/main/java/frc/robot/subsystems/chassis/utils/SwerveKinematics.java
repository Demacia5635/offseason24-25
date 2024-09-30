package frc.robot.subsystems.chassis.utils;

import org.opencv.core.Scalar;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;

/**
 * Class to enhance WPILIB SwerveDriveKinematics
 * 
 * In sewerveToModuleState
 * - if omega is in low range - increase the rate
 * - if omega is non zero - change the vx/vy to compensate for the turn
 * 
 * In toChassisSpeeds
 * - do the reverse - if Omega is non zero - change the result vx/vy as
 * compensated
 */
public class SwerveKinematics extends SwerveDriveKinematics {

    public SwerveModuleState[] states;
    private final double MIN_ANGLE_DIFF = 5 * 0.02;
    private final double MAX_ANGLE_DIFF = ChassisConstants.MAX_STEER_VELOCITY * 0.02; //CHANGE TO REAL LIMIT


    Translation2d[] moduleTranslationsMeters;
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    /**
     * Constructor we use
     * 
     * @param moduleTranslationsMeters
     */
    public SwerveKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
        this.moduleTranslationsMeters = moduleTranslationsMeters;

    }

    private double getCycleDistance(double vel) {
        return vel * 0.02;
    }

    /**
     * Rotate the speeds counter to omega - to drive stright
     */

    public SwerveModuleState calcStateLine(SwerveModuleState prev, Pose2d estimatedPose, Pose2d curPose, ChassisSpeeds speeds){
        double distance = estimatedPose.minus(curPose).getTranslation().getNorm();
        return new SwerveModuleState(distance / 0.02, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getAngle());
    }

    
    public SwerveModuleState calcStateCurve(Rotation2d alpha, Pose2d estimatedPose, SwerveModuleState prevState, Pose2d curPose, Translation2d moduleLocationDifference){
        if(Math.abs(alpha.getDegrees())>=MAX_ANGLE_DIFF)alpha=new Rotation2d(Math.toRadians(MAX_ANGLE_DIFF*Math.abs(alpha.getRadians())/alpha.getRadians()));
        double radius =  (moduleLocationDifference.getNorm() * Math.sin((Math.PI / 2) - alpha.getRadians()))  / Math.sin(alpha.getRadians() * 2);
        double moduleV = alpha.times(2 * radius).getRadians() / 0.02; // (2alpha * d * sin(0.5pi - alpha)/sin(2alpha))/0.02 = Vn


        double startingModuleRadians = prevState.angle.getRadians();
        double chassisDiffRadians = estimatedPose.getRotation().minus(curPose.getRotation()).getRadians();

        double moduleAngle = startingModuleRadians + (2 * alpha.getRadians()) - chassisDiffRadians;
        return new SwerveModuleState(moduleV, new Rotation2d(moduleAngle));
    }
    
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, Pose2d curPose, SwerveModuleState[] prevStates) {

        Pose2d estimatedPose = new Pose2d(curPose.getX() + getCycleDistance(speeds.vxMetersPerSecond),
            curPose.getY() + getCycleDistance(speeds.vyMetersPerSecond),
            curPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * 0.02)));  // and direction


       
        SwerveModuleState[] newModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {

            Translation2d moduleEstimatedPos = estimatedPose.getTranslation().plus(
                    moduleTranslationsMeters[i].rotateBy(estimatedPose.getRotation())); 
            
            Translation2d curModulePos = curPose.getTranslation().plus(moduleTranslationsMeters[i].rotateBy(curPose.getRotation()));
           

            Translation2d moduleLocationDifference = moduleEstimatedPos.minus(
                curModulePos); 

            System.out.println("LOCATION DIFF: " + moduleLocationDifference);
            Rotation2d alpha = moduleLocationDifference.getAngle()
                    .minus(prevStates[i].angle.plus(curPose.getRotation())); 
            System.out.println("alpha: " + alpha.getDegrees());
            
            
            newModuleStates[i] = Math.abs(alpha.getDegrees()) >= MIN_ANGLE_DIFF
            ? calcStateCurve(alpha, estimatedPose, prevStates[i], curPose, moduleLocationDifference) 
            : calcStateLine(prevStates[i], estimatedPose, curPose, speeds);
            lastSpeeds = speeds;
        }
        System.out.println("("+newModuleStates[0].speedMetersPerSecond+","+newModuleStates[0].angle+")");
        System.out.println("("+newModuleStates[1].speedMetersPerSecond+","+newModuleStates[1].angle+")");
        System.out.println("("+newModuleStates[2].speedMetersPerSecond+","+newModuleStates[2].angle+")");
        System.out.println("("+newModuleStates[3].speedMetersPerSecond+","+newModuleStates[3].angle+")");


        return newModuleStates;
    }


    /**
     * Rotate the speeds back to omega - to drive stright
     */
    @Override
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        return new ChassisSpeeds();
    }

}