package frc.robot.vision.utils;


import static frc.robot.vision.ConstantsVision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
public class TagPoseCalc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tagYaw;
    private double tagPitch;
    private double id;
    private double sumdegry;
    private Rotation2d gyroYaw;
    public boolean isIedMostly;

    public TagPoseCalc(double tagYaw, double tagPitch, double x_offset, double y_offset, double id,Rotation2d gyroYaw) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tagYaw = -tagYaw;
        this.tagPitch = tagPitch;
        this.gyroYaw = gyroYaw;
        this.height = TAG_HIGHT[(int)id];
    }
    public void updatePosValues(double tagYaw, double tagPitch, double x_offset, double y_offset, double id,Rotation2d gyroYaw) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tagYaw = -tagYaw;
        this.tagPitch = tagPitch;
        this.gyroYaw = gyroYaw;
        this.height = TAG_HIGHT[(int)id];
    }


    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        sumdegry = tagPitch + TAG_LIMELIGHT_ANGLE;
        
        double robotRelativDist = (Math.abs(height - TAG_LIMELIGHT_HEIGHT)) / (Math.tan(Math.toRadians(sumdegry)));
       
        double dist = robotRelativDist*Math.cos(tagYaw);
        
        return (dist);
    }




    // Get object identifier (note or AprilTag)
    public String translateIdToHashmap() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    public Translation2d getRobotToTag() {

        Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), 
            Rotation2d.fromDegrees(tagYaw));
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset);//.rotateBy(gyroYaw.unaryMinus());
        Translation2d robotToTag = robotToCamera.plus(cameraToTag);

        return robotToTag;
    }



    // public Translation2d getTagToRobot() {
    //     Translation2d robotToCamera = new Translation2d(x_offset, y_offset);
    //     Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx));
    //     Translation2d tagToRobot = robotToCamera.plus(cameraToTag);
    //     return tagToRobot;
    // }


    //get position of robot on the field origin is the point (0,0)!!!!
    public Pose2d calculatePose() {
        Translation2d originToRobot;
        Translation2d toTag;
        Rotation2d tagA;

        toTag = ORIGON_TO_TAG[(int)this.id];
        tagA = Rotation2d.fromDegrees(TAG_ANGLE[(int)this.id]);

        if(toTag != null){
            Translation2d robotToTag = getRobotToTag();
            Rotation2d rot = robotToTag.getAngle();
            Rotation2d rot1 = rot.plus(gyroYaw);
            Rotation2d rot2 = tagA.minus(rot1).rotateBy(Rotation2d.fromDegrees(180)).unaryMinus();
            double norm = robotToTag.getNorm();
//            System.out.println("-------------------------------");
//            System.out.println("toTag = " + rot.getDegrees() + " gyro=" + 
//                    gyroYaw.getDegrees() + " fldToTag=" + rot1.getDegrees() + " tagTo = " + rot2.getDegrees());
//            System.out.println("-------------------------------");
            originToRobot = toTag.plus(new Translation2d(norm, rot2));
            return new Pose2d(originToRobot, gyroYaw);
        }
        return new Pose2d();
    }

}
//.rotateBy(Rotation2d.fromDegrees(isTagRed[(int)this.id]? 180:0))