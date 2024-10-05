
package frc.robot.chassis.utils;

import static frc.robot.chassis.ChassisConstants.CYCLE_DT;

import org.opencv.core.Scalar;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.chassis.ChassisConstants;

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
    private final double MIN_ANGLE_DIFF = 5;
    private final double MAX_ANGLE_DIFF = ChassisConstants.MAX_STEER_VELOCITY * CYCLE_DT;


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

    private double getCycleDistance(double vel) {
        return vel * CYCLE_DT;
    }


    public SwerveModuleState calcStateLine(Pose2d estimatedPose, Pose2d curPose, ChassisSpeeds speeds){
        System.out.println("LINE");
        double distance = estimatedPose.minus(curPose).getTranslation().getNorm();
        Translation2d velocityVector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        // TODO check if speed is more then the max 
        return SwerveModuleState.optimize(
            new SwerveModuleState(
                (distance / CYCLE_DT),
                velocityVector.getAngle()),
                 curPose.getRotation()
                );
    }

    private boolean isVelocityPositive(Translation2d velocityVector){
        return velocityVector.getAngle().getDegrees() >= -90 && velocityVector.getAngle().getDegrees() <= 90;

    }
    
    public SwerveModuleState calcStateCurve(Rotation2d alpha, Pose2d estimatedPose, SwerveModuleState prevState, Pose2d curPose, Translation2d moduleLocationDifference){
        if(Math.abs(alpha.getDegrees())>=MAX_ANGLE_DIFF) alpha = new Rotation2d(Math.toRadians(MAX_ANGLE_DIFF*Math.signum(alpha.getDegrees())));
        System.out.println("CURVE");
        double radius =  (moduleLocationDifference.getNorm() * Math.sin((Math.PI / 2) - alpha.getRadians()))  / Math.sin(alpha.getRadians() * 2);
        double moduleV = alpha.times(2).getRadians() * radius / CYCLE_DT; // (2alpha * d * sin(0.5pi - alpha)/sin(2alpha))/0.02 = Vn


        double startingModuleRadians = prevState.angle.getRadians();
        double chassisDiffRadians = estimatedPose.getRotation().minus(curPose.getRotation()).getRadians();

        double moduleAngle = startingModuleRadians + (2 * alpha.getRadians()) - chassisDiffRadians;
        return SwerveModuleState.optimize(new SwerveModuleState(moduleV, new Rotation2d(moduleAngle)), curPose.getRotation());
    }
    
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, Pose2d curPose, SwerveModuleState[] prevStates) {

        Pose2d estimatedPose = new Pose2d(curPose.getX() + getCycleDistance(speeds.vxMetersPerSecond),
            curPose.getY() + getCycleDistance(speeds.vyMetersPerSecond),
            curPose.getRotation().plus(new Rotation2d(getCycleDistance(speeds.omegaRadiansPerSecond))));  // and direction


       
        SwerveModuleState[] newModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {

            Translation2d moduleEstimatedPos = estimatedPose.getTranslation().plus(
                    moduleTranslationsMeters[i].rotateBy(estimatedPose.getRotation())); 
            
            Translation2d curModulePos = curPose.getTranslation().plus(moduleTranslationsMeters[i].rotateBy(curPose.getRotation()));
           

            Translation2d moduleLocationDifference = moduleEstimatedPos.minus(
                curModulePos); 

            Rotation2d alpha = moduleLocationDifference.getAngle()
                    .minus(prevStates[i].angle); 

            

            if(Math.abs(speeds.vxMetersPerSecond) <= 0.05
                && Math.abs(speeds.vyMetersPerSecond) <= 0.05
                && Math.abs(speeds.omegaRadiansPerSecond) <= 0.05) {
                    newModuleStates[i] = new SwerveModuleState(0, prevStates[i].angle);
            }
            else{
                newModuleStates[i] = Math.abs(alpha.getDegrees()) >= MIN_ANGLE_DIFF
                ? calcStateCurve(alpha, estimatedPose, prevStates[i], curPose, moduleLocationDifference) 
                : calcStateLine(estimatedPose, curPose, speeds);
            }
            System.out.println("MODULE STATE " + i + ": " + newModuleStates[i]);
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

}
