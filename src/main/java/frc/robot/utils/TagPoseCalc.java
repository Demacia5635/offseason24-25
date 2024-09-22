package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class TagPoseCalc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;
    private double id;
    private double sumdegry;
    private Pose2d point;
    private double robotYaw;
    public boolean isIedMostly;


    public TagPoseCalc(double tx, double ty, double x_offset, double y_offset, double id, double robotYaw) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        this.height = Constants.HEIGHT_MAP.get(id);
        this.robotYaw = robotYaw;
    }
    public boolean checkIfRightRotation(){
        switch ((int)this.id) {
            case 7,8,9,10,11,14:
                return false;
            //case 7:
                // return false;
                
            // case 8:
            //     return false;
                
            // case 9:
            //     return false;
                
            // case 10:
            //     return false;
                
            // case 11:
            //     return false;
            // case 12:
            //     return false;                             
        }
        return true;
    }
    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        sumdegry = ty + Constants.LimelightAngle;
        sumdegry = Math.toRadians(sumdegry);
        return ((Math.abs(height - Constants.LimelightHight)) / (Math.tan(sumdegry)));
    }


    // Calculate distance FROM ROBOT CENTER TO TAG
    public double GetDistFromRobotCenter() {
        Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx));
        Translation2d offsetVector = new Translation2d(x_offset, y_offset);
        Translation2d robotToTag = offsetVector.plus(cameraToTag);
        return robotToTag.getNorm();
    }

    // Get object identifier (note or AprilTag)
    //TODO: change name
    public String GetObj() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    // Calculate angle to the object from the ROBOT CENTER
    public Rotation2d getYawFromRobotCenter() {
        //TODO: add TAG angle, if blue add 180 AND tag angle
        Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx));
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset);
        Translation2d robotToTag = cameraToTag.plus(robotToCamera);
        return robotToTag.getAngle();
    }

    //get position of robot on the field
    public Pose2d calculatePose() {
        //TODO: change dic to a more desctiptive name
        Translation2d originToTag = Constants.dic.get(this.GetObj());
        if(originToTag != null){
            Translation2d robotToTag = new Translation2d(GetDistFromRobotCenter(),getYawFromRobotCenter());
            Translation2d originToRobot = originToTag.minus(robotToTag);
            point = new Pose2d(originToRobot,getYawFromRobotCenter());
            return point;
        }
        return point;
    }

}