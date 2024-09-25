package frc.robot.utils.ImageProsesing;

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

    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        sumdegry = ty + Constants.LimelightAngle;
        sumdegry = Math.toRadians(sumdegry);
        return ((Math.abs(height - Constants.LimelightHight)) / (Math.tan(sumdegry)));
    }




    // Get object identifier (note or AprilTag)
    public String translateIdToHashmap() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    // Calculate vector Camera to tag
    public Translation2d getRobotToTag() {
        Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx)).rotateBy(giroYaw);

        
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset).rotateBy(Rotation2d.fromDegrees(Constants.LimelightYaw).plus(giroYaw));
        Translation2d robotToTag = cameraToTag.plus(robotToCamera);
        return robotToTag;
    }

    //get position of robot on the field origin is the point (0,0)!!!!
    public Pose2d calculatePose() {
        Translation2d originToRobot;
        Translation2d originToTag = Constants.CARTESIANVECTORS_MAP.get(this.translateIdToHashmap());
        if(originToTag != null){
            Translation2d robotToTag = getRobotToTag();
            originToRobot = originToTag.plus(robotToTag);
            

            point = new Pose2d(originToRobot,giroYaw);

        }
        return point;
    }

}