package frc.robot.subsystems.vision.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.utils.UpdatedPoseEstimatorClasses.SwerveDrivePoseEstimator;
import frc.robot.utils.Utils;

import static frc.robot.subsystems.vision.VisionConstants.*;
//object to save vision data that includes a pose a timestamp and the difference at that moment from the odometrey

public class VisionData {
    
    //#region props
    private Pose2d pose;
    private double timeStamp;
    private double diffrence; // difference than odometry
    private Pose2d notFilteredPose;
    private double notFilteredtimeStamp;
    private SwerveDrivePoseEstimator bufPoseEstimator;
    //#endregion

    //#region C'tor
    public VisionData(Pose2d pose, double timeStamp, SwerveDrivePoseEstimator bufPoseEstimator) {
        this.pose = pose;
        this.timeStamp = timeStamp;
        this.notFilteredPose = pose;
        this.notFilteredtimeStamp = timeStamp;

        this.bufPoseEstimator = bufPoseEstimator;
        if (timeStamp < 0) {
            System.out.println("cleared at constructor, " + timeStamp);
            clear();
            
        } else {
            
            setDiffrence();
        }
    }
    //#endregion

    //#region util methods

    private void setDiffrence() {

        Pose2d poseSample = this.bufPoseEstimator.getSample(timeStamp);
        if (poseSample != null 
        && Math.abs(Utils.angelErrorInDegrees(poseSample.getRotation(),pose.getRotation(),0)) < maxValidAngleDiff) {
            
            diffrence = poseSample.getTranslation().getDistance(pose.getTranslation());
        } 
        else {
            if(poseSample != null){
               // System.out.println("cleared on setDifference() func pose sample isnt null, " + poseSample.getRotation().getDegrees() + " " + pose.getRotation().getDegrees());
                clear();
            }
            else{
                //System.out.println("cleared on setDifference() func pose sample is null estimator pose = " + bufPoseEstimator.getEstimatedPosition());
                clear();
            }
        }
    }

    public void clear() {
        diffrence = -1;
        pose = null;
        timeStamp = 0;
        //System.out.println("Cleared.");
    }    
    //#endregion

    //#region getters

    public Pose2d getPose() {
        return this.pose;
    }

    public double getTimeStamp() {
        return this.timeStamp;
    }

    public double getDiffrence() {
        return this.diffrence;
    }

    public Pose2d getNotFilteredPose() {
        return this.notFilteredPose;
    }

    public double getNotFilteredtimeStamp() {
        return this.notFilteredtimeStamp;
    }
    //#endregion

    
}