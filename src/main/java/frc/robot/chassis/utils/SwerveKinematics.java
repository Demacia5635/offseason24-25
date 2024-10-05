
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final double MAX_ANGLE_DIFF = 30;


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
        double radius =  (moduleLocationDifference.getNorm() * Math.sin((Math.PI / 2) - Math.abs(alpha.getRadians())))  / Math.sin(alpha.getRadians() * 2);
        double moduleV = alpha.times(2).getRadians() * radius / CYCLE_DT; // (2alpha * d * sin(0.5pi - alpha)/sin(2alpha))/0.02 = Vn


        double startingModuleRadians = prevState.angle.getRadians();

        
        double moduleAngle = startingModuleRadians + (2 * alpha.getRadians());
        return SwerveModuleState.optimize(new SwerveModuleState(moduleV, new Rotation2d(moduleAngle)), curPose.getRotation());
    }
    
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, Pose2d curPose, SwerveModuleState[] prevStates) {
        //looks ok
        Pose2d estimatedPose = new Pose2d(curPose.getX() + getCycleDistance(speeds.vxMetersPerSecond),
            curPose.getY() + getCycleDistance(speeds.vyMetersPerSecond),
            curPose.getRotation().plus(new Rotation2d(getCycleDistance(speeds.omegaRadiansPerSecond))));  // and direction


       
        SwerveModuleState[] newModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            //looks ok
            Translation2d moduleEstimatedPos = estimatedPose.getTranslation().plus(
                    moduleTranslationsMeters[i].rotateBy(estimatedPose.getRotation())); 
            //looks ok
            Translation2d curModulePos = curPose.getTranslation().plus(moduleTranslationsMeters[i].rotateBy(curPose.getRotation()));
           
            //looks ok
            Translation2d moduleLocationDifference = moduleEstimatedPos.minus(
                curModulePos); 
            //looks right - might be different sign?
            Rotation2d alpha = moduleLocationDifference.getAngle()
                    .minus(prevStates[i].angle); 

            SmartDashboard.putNumber("Alpha" + i, alpha.getDegrees());
            SmartDashboard.putNumber("Start" + i, prevStates[i].angle.getDegrees());

            if(Math.abs(speeds.vxMetersPerSecond) <= 0.05
                && Math.abs(speeds.vyMetersPerSecond) <= 0.05
                && Math.abs(speeds.omegaRadiansPerSecond) <= 0.05) {
                    newModuleStates[i] = new SwerveModuleState(0, prevStates[i].angle);
            }
            else{
                newModuleStates[i] = Math.abs(alpha.getDegrees()) >= MIN_ANGLE_DIFF
                ? calcStateCurve(alpha, estimatedPose, prevStates[i], curPose, moduleLocationDifference) 
                : calcStateLine(estimatedPose, curPose, speeds);
            SmartDashboard.putBoolean("curve?" + i, Math.abs(alpha.getDegrees()) >= MIN_ANGLE_DIFF);
            }
        }
        
        System.out.println("MODULE STATE 0 "  + 0 + ": " + newModuleStates[0]);
        SmartDashboard.putNumber("ModuleState", newModuleStates[0].angle.getDegrees());
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
