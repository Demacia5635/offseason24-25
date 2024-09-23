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
    private Rotation2d giroYaw;
    public boolean isIedMostly;


    public TagPoseCalc(double tx, double ty, double x_offset, double y_offset, double id,Rotation2d giroYaw) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        this.giroYaw = giroYaw;
        this.height = Constants.HEIGHT_MAP.get(id);
    }
    public boolean checkIfAdd180(){
        boolean isblueAndNot6 = false;
        switch ((int)this.id) {
            case 7,8,9,10,14,15,16:
                isblueAndNot6 = true;
                break;                
        }
        return isblueAndNot6;
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
    public String translateIdToHashmap() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    // Calculate angle to the object from the ROBOT CENTER
    public Rotation2d getYawFromRobotCenter() {
        Translation2d cameraToTag;
        if (checkIfAdd180()){
            cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx+Constants.IDTOANGLE_MAP.get(translateIdToHashmap())+180));
        }
        else{
            cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx+Constants.IDTOANGLE_MAP.get(translateIdToHashmap())));
        }
        
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset);
        Translation2d robotToTag = cameraToTag.plus(robotToCamera);
        return robotToTag.getAngle();
    }

    //get position of robot on the field origin is the point (0,0)!!!!
    public Pose2d calculatePose() {
        System.out.println("get dist in m: "+GetDistFromRobotCenter());
        System.out.println("get angle yaw from robot: "+giroYaw);
        Translation2d originToRobot;
        Translation2d originToTag = Constants.CARTESIANVECTORS_MAP.get(this.translateIdToHashmap());
        if(originToTag != null){
            Translation2d robotToTag = new Translation2d(GetDistFromRobotCenter(),giroYaw);
            if (checkIfAdd180()){
                originToRobot = originToTag.plus(robotToTag);
            }
            else{
                originToRobot = originToTag.minus(robotToTag);
            }
            point = new Pose2d(originToRobot,giroYaw);

        }
        return point;
    }

}