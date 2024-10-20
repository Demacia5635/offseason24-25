package frc.robot.vision.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.vision.utils.ConstantsVision.*;


public class NotePoseCalc {
    private double x_offset;
    private double y_offset;
    private double angleYaw;
    private double noteYaw;
    private Pose2d pose;
    private Rotation2d gyroYaw;


    public NotePoseCalc(double noteYaw, double x_offset, double y_offset,Pose2d pose) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.pose = pose;
        this.gyroYaw = pose.getRotation();
    }
    public void update(double noteYaw,double x_offset, double y_offset,Pose2d pose) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;

        this.pose = pose;
        this.gyroYaw = pose.getRotation();
    }

    



    // //Calculate distance FROM CAMERA TO TAG
    // public double GetDistFromCamera() {
    //     angleYaw = Math.toRadians(widthInAngle);
    //     double sideDist = (widthInPix*Math.sin((Math.PI/2)-(angleYaw/2)))/Math.sin(angleYaw);
    //     double distInPix = sideDist/Math.cos(angleYaw/2);
    //     double meterToPix = (2*NOTE_RIDIUS)/widthInPix;
    //     double dist = (distInPix*meterToPix)*Math.cos(Math.toRadians(noteYaw));
    //     return (dist);

    // }
    public double GetDistFromCamera() {
        return 1;
    }



    // Calculate vector Camera to tag
    public Translation2d getRobotToNote() {
        Translation2d cameraToNote = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(noteYaw)).rotateBy(gyroYaw.unaryMinus());
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset).rotateBy(gyroYaw.unaryMinus());
        Translation2d robotToNote = robotToCamera.plus(cameraToNote);
        return robotToNote;
    }

    //get position of robot on the field origin is the pose (0,0)!!!!
    public Pose2d calculatePose() {
        // Translation2d originToNote;
        
        // if(widthInAngle!= 0 && pose != null){
        //     Translation2d originToRobot = pose.getTranslation();
        //     Translation2d robotToNote = getRobotToNote();
        //     originToNote = originToRobot.plus(robotToNote);
            

        //     pose = new Pose2d(originToNote,new Rotation2d());
        //     return pose;
        // }
        return new Pose2d();
    }

}